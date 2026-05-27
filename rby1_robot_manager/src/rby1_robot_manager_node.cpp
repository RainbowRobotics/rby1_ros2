#include <algorithm>
#include <cctype>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rby1_hardware_interface/rby1_robot_client.hpp"
#include "rby1_msgs/msg/robot_state.hpp"
#include "rby1_msgs/msg/tool_flange_state.hpp"
#include "rby1_msgs/srv/control_manager_command.hpp"
#include "rby1_msgs/srv/gravity_compensation.hpp"
#include "rby1_msgs/srv/state_on_off.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace rby1_robot_manager
{
namespace
{

using rby1_hardware_interface::CommandResult;
using rby1_hardware_interface::ControlManagerCommand;
using rby1_hardware_interface::RobotClientOptions;
using rby1_hardware_interface::RobotStateSnapshot;
using rby1_hardware_interface::Rby1RobotClient;

std::string trim(const std::string & value)
{
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char character) {
    return std::isspace(character) != 0;
  });
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char character) {
    return std::isspace(character) != 0;
  }).base();
  if (first >= last) {
    return "";
  }
  return std::string(first, last);
}

std::vector<std::string> split_selector(const std::string & value)
{
  std::vector<std::string> tokens;
  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ',')) {
    const auto trimmed = trim(token);
    if (!trimmed.empty()) {
      tokens.push_back(trimmed);
    }
  }
  return tokens;
}

std::string join_regex(const std::vector<std::string> & tokens)
{
  std::string result;
  for (size_t index = 0; index < tokens.size(); ++index) {
    result += tokens[index];
    if (index + 1 < tokens.size()) {
      result += "|";
    }
  }
  return result;
}

std::string normalize_power_selector(const std::string & selector)
{
  const auto tokens = split_selector(selector.empty() ? "all" : selector);
  std::vector<std::string> regex_tokens;
  regex_tokens.reserve(tokens.size());
  for (const auto & token : tokens) {
    if (token == "all" || token == ".*") {
      return ".*";
    }
    if (token == "5" || token == "12" || token == "24" || token == "48") {
      regex_tokens.push_back(token + "v");
    } else {
      regex_tokens.push_back(token);
    }
  }
  return regex_tokens.empty() ? ".*" : join_regex(regex_tokens);
}

std::string normalize_servo_selector(const std::string & selector)
{
  const auto tokens = split_selector(selector.empty() ? "all" : selector);
  std::vector<std::string> regex_tokens;
  regex_tokens.reserve(tokens.size());
  for (const auto & token : tokens) {
    if (token == "all" || token == ".*") {
      return ".*";
    }
    if (token == "right" || token == "right_arm") {
      regex_tokens.push_back("^right_arm_.*");
    } else if (token == "left" || token == "left_arm") {
      regex_tokens.push_back("^left_arm_.*");
    } else if (token == "torso") {
      regex_tokens.push_back("^torso_.*");
    } else if (token == "head") {
      regex_tokens.push_back("^head_.*");
    } else {
      regex_tokens.push_back(token);
    }
  }
  return regex_tokens.empty() ? ".*" : join_regex(regex_tokens);
}

int parse_tool_voltage(const std::string & selector, bool enabled)
{
  if (!enabled) {
    return 0;
  }
  const auto trimmed = trim(selector);
  if (trimmed.empty()) {
    return 12;
  }
  if (trimmed == "12" || trimmed == "12v") {
    return 12;
  }
  if (trimmed == "24" || trimmed == "24v") {
    return 24;
  }
  throw std::invalid_argument("tool flange voltage must be 12 or 24 when enabling");
}

void fill_tool_flange_msg(
  rby1_msgs::msg::ToolFlangeState & message,
  const rby1_hardware_interface::ToolFlangeStatus & status)
{
  message.ft_force = status.ft_force;
  message.ft_torque = status.ft_torque;
  message.gyro = status.gyro;
  message.acceleration = status.acceleration;
  message.switch_a = status.switch_a;
  message.output_voltage = status.output_voltage;
  message.digital_input_a = status.digital_input_a;
  message.digital_input_b = status.digital_input_b;
  message.digital_output_a = status.digital_output_a;
  message.digital_output_b = status.digital_output_b;
}

}  // namespace

class Rby1RobotManagerNode final : public rclcpp::Node
{
public:
  Rby1RobotManagerNode()
  : Node("rby1_robot_manager")
  {
    RobotClientOptions options;
    options.robot_ip = declare_parameter<std::string>("robot_ip", options.robot_ip);
    options.model = declare_parameter<std::string>("model", options.model);
    options.auto_reconnect = declare_parameter<bool>("auto_reconnect", options.auto_reconnect);
    options.connect_timeout_sec = declare_parameter<double>("connect_timeout_sec", options.connect_timeout_sec);
    options.read_timeout_sec = declare_parameter<double>("read_timeout_sec", options.read_timeout_sec);

    connect_on_start_ = declare_parameter<bool>("connect_on_start", true);
    publish_status_ = declare_parameter<bool>("publish_status", true);
    publish_battery_state_ = declare_parameter<bool>("publish_battery_state", true);
    publish_tool_flange_state_ = declare_parameter<bool>("publish_tool_flange_state", true);
    const auto read_period_sec = declare_parameter<double>("status_period_sec", 0.02);

    client_ = std::make_unique<Rby1RobotClient>(options);
    if (connect_on_start_) {
      try_connect();
    }

    robot_state_publisher_ = create_publisher<rby1_msgs::msg::RobotState>("robot_state", 10);
    battery_state_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    tool_flange_left_publisher_ = create_publisher<rby1_msgs::msg::ToolFlangeState>("tool_flange/left", 10);
    tool_flange_right_publisher_ = create_publisher<rby1_msgs::msg::ToolFlangeState>("tool_flange/right", 10);

    robot_power_service_ = create_service<rby1_msgs::srv::StateOnOff>(
      "robot_power",
      [this](const auto request, auto response) { handle_power(request, response); });
    robot_servo_service_ = create_service<rby1_msgs::srv::StateOnOff>(
      "robot_servo",
      [this](const auto request, auto response) { handle_servo(request, response); });
    motor_brake_service_ = create_service<rby1_msgs::srv::StateOnOff>(
      "set_motor_brake",
      [this](const auto request, auto response) { handle_brake(request, response); });
    tool_flange_service_ = create_service<rby1_msgs::srv::StateOnOff>(
      "tool_flange_power",
      [this](const auto request, auto response) { handle_tool_flange(request, response); });
    gravity_compensation_service_ = create_service<rby1_msgs::srv::GravityCompensation>(
      "gravity_compensation",
      [this](const auto request, auto response) { handle_gravity_compensation(request, response); });
    control_manager_service_ = create_service<rby1_msgs::srv::ControlManagerCommand>(
      "control_manager_command",
      [this](const auto request, auto response) { handle_control_manager(request, response); });

    if (publish_status_) {
      const auto period = std::chrono::duration<double>(std::max(0.005, read_period_sec));
      status_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { publish_status(); });
    }
  }

private:
  bool ensure_connected()
  {
    if (client_->is_connected()) {
      return true;
    }
    return try_connect();
  }

  bool try_connect()
  {
    try {
      client_->connect();
      RCLCPP_INFO(get_logger(), "Connected to RBY1 robot at %s", client_->options().robot_ip.c_str());
      return true;
    } catch (const std::exception & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Failed to connect to RBY1 robot: %s",
        exception.what());
      return false;
    }
  }

  template <typename RequestT, typename ResponseT, typename CallbackT>
  void execute_service(
    const std::shared_ptr<RequestT> request,
    const std::shared_ptr<ResponseT> response,
    CallbackT callback)
  {
    std::lock_guard<std::mutex> lock(client_mutex_);
    try {
      if (!ensure_connected()) {
        response->success = false;
        response->message = "robot client is not connected";
        return;
      }
      const auto command_result = callback(request);
      response->success = command_result.success;
      response->message = command_result.message;
    } catch (const std::exception & exception) {
      response->success = false;
      response->message = exception.what();
    }
  }

  void handle_power(
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response)
  {
    execute_service(request, response, [this](const auto request_handle) {
      return client_->set_power(normalize_power_selector(request_handle->parameters), request_handle->state);
    });
  }

  void handle_servo(
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response)
  {
    execute_service(request, response, [this](const auto request_handle) {
      return client_->set_servo(normalize_servo_selector(request_handle->parameters), request_handle->state);
    });
  }

  void handle_brake(
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response)
  {
    execute_service(request, response, [this](const auto request_handle) {
      const auto selector = normalize_servo_selector(request_handle->parameters);
      return client_->set_brake(selector, request_handle->state);
    });
  }

  void handle_tool_flange(
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response)
  {
    execute_service(request, response, [this](const auto request_handle) {
      return client_->set_tool_flange_output_voltage(
        parse_tool_voltage(request_handle->parameters, request_handle->state));
    });
  }

  void handle_gravity_compensation(
    const std::shared_ptr<rby1_msgs::srv::GravityCompensation::Request> request,
    const std::shared_ptr<rby1_msgs::srv::GravityCompensation::Response> response)
  {
    std::lock_guard<std::mutex> lock(client_mutex_);
    try {
      if (!ensure_connected()) {
        response->success = false;
        response->message = "robot client is not connected";
        return;
      }
      const auto command_result = client_->set_gravity_compensation(request->part_name, request->state);
      response->success = command_result.success;
      response->message = command_result.message;
    } catch (const std::exception & exception) {
      response->success = false;
      response->message = exception.what();
    }
  }

  void handle_control_manager(
    const std::shared_ptr<rby1_msgs::srv::ControlManagerCommand::Request> request,
    const std::shared_ptr<rby1_msgs::srv::ControlManagerCommand::Response> response)
  {
    std::lock_guard<std::mutex> lock(client_mutex_);
    try {
      if (!ensure_connected()) {
        response->success = false;
        response->message = "robot client is not connected";
        return;
      }

      ControlManagerCommand command;
      if (request->command == rby1_msgs::srv::ControlManagerCommand::Request::CMD_ENABLE) {
        command = ControlManagerCommand::kEnable;
      } else if (request->command == rby1_msgs::srv::ControlManagerCommand::Request::CMD_DISABLE) {
        command = ControlManagerCommand::kDisable;
      } else if (request->command == rby1_msgs::srv::ControlManagerCommand::Request::CMD_RESET) {
        command = ControlManagerCommand::kResetFault;
      } else {
        response->success = false;
        response->message = "unsupported control manager command";
        return;
      }

      const auto command_result = client_->set_control_manager(command);
      response->success = command_result.success;
      response->message = command_result.message;
    } catch (const std::exception & exception) {
      response->success = false;
      response->message = exception.what();
    }
  }

  void publish_status()
  {
    std::lock_guard<std::mutex> lock(client_mutex_);
    try {
      if (!ensure_connected()) {
        return;
      }
      const auto snapshot = client_->read_state();
      publish_robot_state(snapshot);
      if (publish_battery_state_ && snapshot.battery.available) {
        publish_battery(snapshot);
      }
      if (publish_tool_flange_state_) {
        publish_tool_flange(snapshot);
      }
    } catch (const std::exception & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Failed to publish RBY1 status: %s",
        exception.what());
    }
  }

  void publish_robot_state(const RobotStateSnapshot & snapshot)
  {
    rby1_msgs::msg::RobotState message;
    message.control_manager_state = snapshot.control_manager_state;
    message.brake_state.left_arm = snapshot.brakes.left_arm;
    message.brake_state.right_arm = snapshot.brakes.right_arm;
    message.brake_state.torso = snapshot.brakes.torso;
    message.brake_state.head = snapshot.brakes.head;
    message.tool_flange_state = {snapshot.tool_flange_connected[0], snapshot.tool_flange_connected[1]};
    message.emo_state = snapshot.emo_state;
    message.center_of_mass = snapshot.center_of_mass;
    robot_state_publisher_->publish(message);
  }

  void publish_battery(const RobotStateSnapshot & snapshot)
  {
    sensor_msgs::msg::BatteryState message;
    message.header.stamp = now();
    message.voltage = snapshot.battery.voltage;
    message.current = snapshot.battery.current;
    message.percentage = snapshot.battery.percentage;
    battery_state_publisher_->publish(message);
  }

  void publish_tool_flange(const RobotStateSnapshot & snapshot)
  {
    rby1_msgs::msg::ToolFlangeState left_message;
    fill_tool_flange_msg(left_message, snapshot.left_tool_flange);
    tool_flange_left_publisher_->publish(left_message);

    rby1_msgs::msg::ToolFlangeState right_message;
    fill_tool_flange_msg(right_message, snapshot.right_tool_flange);
    tool_flange_right_publisher_->publish(right_message);
  }

  std::mutex client_mutex_;
  std::unique_ptr<Rby1RobotClient> client_;
  bool connect_on_start_{true};
  bool publish_status_{true};
  bool publish_battery_state_{true};
  bool publish_tool_flange_state_{true};

  rclcpp::Publisher<rby1_msgs::msg::RobotState>::SharedPtr robot_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
  rclcpp::Publisher<rby1_msgs::msg::ToolFlangeState>::SharedPtr tool_flange_left_publisher_;
  rclcpp::Publisher<rby1_msgs::msg::ToolFlangeState>::SharedPtr tool_flange_right_publisher_;
  rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr robot_power_service_;
  rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr robot_servo_service_;
  rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr motor_brake_service_;
  rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr tool_flange_service_;
  rclcpp::Service<rby1_msgs::srv::GravityCompensation>::SharedPtr gravity_compensation_service_;
  rclcpp::Service<rby1_msgs::srv::ControlManagerCommand>::SharedPtr control_manager_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace rby1_robot_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rby1_robot_manager::Rby1RobotManagerNode>());
  rclcpp::shutdown();
  return 0;
}
