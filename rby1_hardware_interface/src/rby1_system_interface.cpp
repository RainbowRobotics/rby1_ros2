#include "rby1_hardware_interface/rby1_system_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rby1_hardware_interface
{
namespace
{

bool parse_bool(const std::string & value, bool fallback)
{
  if (value == "true" || value == "True" || value == "1") {
    return true;
  }
  if (value == "false" || value == "False" || value == "0") {
    return false;
  }
  return fallback;
}

double parse_double(const std::string & value, double fallback)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return fallback;
  }
}

}  // namespace

hardware_interface::CallbackReturn Rby1SystemInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!validate_joint_interfaces(hardware_info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  options_.robot_ip = hardware_parameter_or(hardware_info, "robot_ip", options_.robot_ip);
  options_.model = hardware_parameter_or(hardware_info, "model", options_.model);
  options_.auto_reconnect = parse_bool(
    hardware_parameter_or(hardware_info, "auto_reconnect", options_.auto_reconnect ? "true" : "false"),
    options_.auto_reconnect);
  options_.read_timeout_sec = parse_double(
    hardware_parameter_or(hardware_info, "read_timeout_sec", std::to_string(options_.read_timeout_sec)),
    options_.read_timeout_sec);
  options_.connect_timeout_sec = parse_double(
    hardware_parameter_or(hardware_info, "connect_timeout_sec", std::to_string(options_.connect_timeout_sec)),
    options_.connect_timeout_sec);

  joint_names_.clear();
  joint_index_by_name_.clear();
  joint_names_.reserve(hardware_info.joints.size());
  for (size_t index = 0; index < hardware_info.joints.size(); ++index) {
    const auto & joint = hardware_info.joints[index];
    if (joint_index_by_name_.count(joint.name) != 0) {
      RCLCPP_ERROR(logger_, "Duplicate joint '%s' in ros2_control hardware info", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
    joint_index_by_name_.emplace(joint.name, index);
  }

  positions_.assign(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  velocities_.assign(joint_names_.size(), 0.0);
  efforts_.assign(joint_names_.size(), 0.0);

  RCLCPP_INFO(
    logger_,
    "Initialized RBY1 hardware interface for model '%s' with %zu read-only joints",
    options_.model.c_str(), joint_names_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rby1SystemInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  try {
    options_.model = normalize_model_name(options_.model);
    robot_client_ = std::make_unique<Rby1RobotClient>(options_);
    robot_client_->connect();
    const auto snapshot = robot_client_->read_state();
    if (!apply_state_snapshot(snapshot)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger_, "Failed to configure RBY1 hardware interface: %s", exception.what());
    robot_client_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configured RBY1 hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rby1SystemInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  if (!robot_client_ || !robot_client_->is_connected()) {
    RCLCPP_ERROR(logger_, "Cannot activate RBY1 hardware interface before connecting to the robot");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    const auto snapshot = robot_client_->read_state();
    if (!apply_state_snapshot(snapshot)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger_, "Failed to synchronize state during activation: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  active_ = true;
  read_error_count_ = 0;
  RCLCPP_INFO(logger_, "Activated RBY1 read-only hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rby1SystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  active_ = false;
  RCLCPP_INFO(logger_, "Deactivated RBY1 hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Rby1SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 3);
  for (size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_POSITION, &positions_[index]);
    interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_VELOCITY, &velocities_[index]);
    interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_EFFORT, &efforts_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> Rby1SystemInterface::export_command_interfaces()
{
  return {};
}

hardware_interface::return_type Rby1SystemInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  if (!robot_client_) {
    RCLCPP_ERROR(logger_, "RBY1 robot client is not configured");
    return hardware_interface::return_type::ERROR;
  }

  try {
    const auto snapshot = robot_client_->read_state();
    return apply_state_snapshot(snapshot) ? hardware_interface::return_type::OK :
           hardware_interface::return_type::ERROR;
  } catch (const std::exception & first_exception) {
    ++read_error_count_;
    if (options_.auto_reconnect) {
      try {
        RCLCPP_WARN(
          logger_,
          "RBY1 state read failed (%s). Attempting reconnect.",
          first_exception.what());
        robot_client_->disconnect();
        robot_client_->connect();
        const auto snapshot = robot_client_->read_state();
        return apply_state_snapshot(snapshot) ? hardware_interface::return_type::OK :
               hardware_interface::return_type::ERROR;
      } catch (const std::exception & reconnect_exception) {
        RCLCPP_ERROR(
          logger_,
          "RBY1 reconnect/read failed after %zu read errors: %s",
          read_error_count_, reconnect_exception.what());
        return hardware_interface::return_type::ERROR;
      }
    }

    RCLCPP_ERROR(
      logger_,
      "RBY1 state read failed after %zu read errors: %s",
      read_error_count_, first_exception.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type Rby1SystemInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  return hardware_interface::return_type::OK;
}

bool Rby1SystemInterface::validate_joint_interfaces(
  const hardware_interface::HardwareInfo & hardware_info) const
{
  if (hardware_info.joints.empty()) {
    RCLCPP_ERROR(logger_, "RBY1 ros2_control hardware info must contain at least one joint");
    return false;
  }

  for (const auto & joint : hardware_info.joints) {
    if (!joint.command_interfaces.empty()) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s' declares command interfaces, but this version is read-only",
        joint.name.c_str());
      return false;
    }

    std::set<std::string> state_interfaces;
    for (const auto & state_interface : joint.state_interfaces) {
      state_interfaces.insert(state_interface.name);
    }

    for (const auto & required_interface : {
        hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_EFFORT})
    {
      if (state_interfaces.count(required_interface) == 0) {
        RCLCPP_ERROR(
          logger_,
          "Joint '%s' must declare state interface '%s'",
          joint.name.c_str(), required_interface);
        return false;
      }
    }
  }

  return true;
}

bool Rby1SystemInterface::apply_state_snapshot(const RobotStateSnapshot & snapshot)
{
  for (const auto & joint_name : joint_names_) {
    const auto snapshot_joint = snapshot.joints.find(joint_name);
    if (snapshot_joint == snapshot.joints.end()) {
      RCLCPP_ERROR(
        logger_,
        "Robot SDK state did not include joint '%s'. Check model and URDF joint names.",
        joint_name.c_str());
      return false;
    }

    const auto index = joint_index_by_name_.at(joint_name);
    const auto & sample = snapshot_joint->second;
    if (!std::isfinite(sample.position) || !std::isfinite(sample.velocity) || !std::isfinite(sample.effort)) {
      RCLCPP_ERROR(logger_, "Robot SDK returned non-finite state for joint '%s'", joint_name.c_str());
      return false;
    }
    positions_[index] = sample.position;
    velocities_[index] = sample.velocity;
    efforts_[index] = sample.effort;
  }

  return true;
}

std::string Rby1SystemInterface::hardware_parameter_or(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & key,
  const std::string & fallback) const
{
  const auto parameter = hardware_info.hardware_parameters.find(key);
  if (parameter == hardware_info.hardware_parameters.end()) {
    return fallback;
  }
  return parameter->second;
}

}  // namespace rby1_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  rby1_hardware_interface::Rby1SystemInterface,
  hardware_interface::SystemInterface)
