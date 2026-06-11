#include "rby1_hardware/rby1_system_hardware.hpp"

#include <chrono>
#include <thread>
#include <algorithm>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rby1_hardware {

hardware_interface::CallbackReturn RBY1SystemHardware::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse parameters from URDF ros2_control configuration
  auto ip_it = info_.hardware_parameters.find("robot_ip");
  if (ip_it != info_.hardware_parameters.end()) {
    robot_ip_ = ip_it->second;
  } else {
    robot_ip_ = "127.0.0.1:50051";
  }

  auto model_it = info_.hardware_parameters.find("model");
  if (model_it != info_.hardware_parameters.end()) {
    model_type_ = model_it->second;
  } else {
    model_type_ = "a";
  }

  auto col_check_it = info_.hardware_parameters.find("collision_check_enable");
  if (col_check_it != info_.hardware_parameters.end()) {
    collision_check_enable_ = (col_check_it->second == "true" || col_check_it->second == "1");
  } else {
    collision_check_enable_ = false;
  }

  auto col_thresh_it = info_.hardware_parameters.find("collision_threshold");
  if (col_thresh_it != info_.hardware_parameters.end()) {
    try {
      collision_threshold_ = std::stod(col_thresh_it->second);
    } catch (...) {
      collision_threshold_ = 0.01;
    }
  } else {
    collision_threshold_ = 0.01;
  }

  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"),
              "Initializing RBY1SystemHardware on %s (Model: %s, Collision Check: %s, Threshold: %.4f m)",
              robot_ip_.c_str(), model_type_.c_str(), 
              collision_check_enable_ ? "ON" : "OFF", collision_threshold_);

  // Resize internal buffers for joints
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  joint_name_to_sdk_index_.resize(info_.joints.size(), 0);

  // Check joint interface configuration
  for (const auto & joint : info_.joints) {
    // Check command interfaces
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                   "Joint '%s' has %zu command interfaces. Expected exactly 1 (position).",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                   "Joint '%s' command interface is '%s'. Expected '%s' (position).",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check state interfaces
    if (joint.state_interfaces.size() < 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                   "Joint '%s' has %zu state interfaces. Expected at least 1 (position).",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                   "Joint '%s' first state interface is '%s'. Expected '%s' (position).",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RBY1SystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RBY1SystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RBY1SystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"), "Activating RBY1 Hardware Interface...");

  // Instantiate SDK Wrapper based on model parameter
  if (model_type_ == "a" || model_type_ == "A") {
    robot_ = std::make_unique<RBY1RobotWrapperImpl<rb::y1_model::A>>();
  } else if (model_type_ == "m" || model_type_ == "M") {
    robot_ = std::make_unique<RBY1RobotWrapperImpl<rb::y1_model::M>>();
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"), "Unsupported robot model: %s", model_type_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Connect to physical robot/simulator via SDK
  if (!robot_->connect(robot_ip_)) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"), "Failed to connect to RBY1 SDK at %s", robot_ip_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"), "Connected to RBY1 SDK successfully.");

  auto robot_info = robot_->get_robot_info();

  // Map URDF joint names to SDK index positions
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint_name = info_.joints[i].name;
    auto joint_it = std::find_if(robot_info.joint_infos.begin(), robot_info.joint_infos.end(),
                                 [&joint_name](const rb::JointInfo & info) {
                                   return info.name == joint_name;
                                 });

    if (joint_it == robot_info.joint_infos.end()) {
      RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                   "Joint '%s' declared in URDF was not found in robot SDK joint list.",
                   joint_name.c_str());
      robot_->disconnect();
      return hardware_interface::CallbackReturn::ERROR;
    }

    unsigned int sdk_idx = std::distance(robot_info.joint_infos.begin(), joint_it);
    joint_name_to_sdk_index_[i] = sdk_idx;
  }

  // Power On and Servo On
  std::string power_dev = "all";
  std::string servo_dev = "all";
  auto power_dev_it = info_.hardware_parameters.find("power_on");
  if (power_dev_it != info_.hardware_parameters.end()) {
    power_dev = power_dev_it->second;
  }
  auto servo_dev_it = info_.hardware_parameters.find("servo_on");
  if (servo_dev_it != info_.hardware_parameters.end()) {
    servo_dev = servo_dev_it->second;
  }

  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"), "Powering ON device: %s...", power_dev.c_str());
  robot_->power_on(power_dev);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"), "Servo ON device: %s...", servo_dev.c_str());
  if (!robot_->servo_on(servo_dev)) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"), "Failed to enable Servo ON for device: %s", servo_dev.c_str());
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Enable Control Manager
  if (!robot_->enable_control_manager()) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"), "Failed to enable SDK Control Manager.");
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!robot_->wait_for_control_ready(2000)) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"), "Control Manager failed to become ready in time.");
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize command stream
  if (!robot_->init_stream()) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"), "Failed to initialize gRPC Command Stream.");
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Populate initial state
  std::vector<double> sdk_positions, sdk_velocities, sdk_torques;
  robot_->get_joint_states(sdk_positions, sdk_velocities, sdk_torques);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    unsigned int sdk_idx = joint_name_to_sdk_index_[i];
    hw_positions_[i] = sdk_positions[sdk_idx];
    hw_velocities_[i] = sdk_velocities[sdk_idx];
    hw_commands_[i] = hw_positions_[i]; // Commanded targets initialized to current positions
  }

  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"), "RBY1 Hardware Interface activated successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RBY1SystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"), "Deactivating RBY1 Hardware Interface...");

  if (robot_) {
    robot_->close_stream();
    robot_->disable_control_manager();
    robot_->cancel_control();
    
    // Disable Servos
    std::string servo_dev = "all";
    auto servo_dev_it = info_.hardware_parameters.find("servo_on");
    if (servo_dev_it != info_.hardware_parameters.end()) {
      servo_dev = servo_dev_it->second;
    }
    robot_->servo_on(servo_dev); // (ServoOff call is not exposed directly in wrapper except via disconnecting or servo off)
    
    robot_->disconnect();
    robot_.reset();
  }

  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"), "RBY1 Hardware Interface deactivated successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RBY1SystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (!robot_ || !robot_->is_connected()) {
    return hardware_interface::return_type::ERROR;
  }

  std::vector<double> sdk_positions, sdk_velocities, sdk_torques;
  robot_->get_joint_states(sdk_positions, sdk_velocities, sdk_torques);

  if (sdk_positions.size() < (size_t)robot_->get_dof()) {
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    unsigned int sdk_idx = joint_name_to_sdk_index_[i];
    hw_positions_[i] = sdk_positions[sdk_idx];
    hw_velocities_[i] = sdk_velocities[sdk_idx];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RBY1SystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (!robot_ || !robot_->is_connected()) {
    return hardware_interface::return_type::ERROR;
  }

  // Construct target positions vector ordered by SDK indices
  int dof = robot_->get_dof();
  std::vector<double> sdk_target_positions(dof, 0.0);

  // Initialize uncontrolled joints to current states (retrieved during last read)
  std::vector<double> sdk_positions, sdk_velocities, sdk_torques;
  robot_->get_joint_states(sdk_positions, sdk_velocities, sdk_torques);
  if (sdk_positions.size() == (size_t)dof) {
    sdk_target_positions = sdk_positions;
  }

  // Fill in active commands from ros2_control
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    unsigned int sdk_idx = joint_name_to_sdk_index_[i];
    sdk_target_positions[sdk_idx] = hw_commands_[i];
  }

  // Predictive collision checking
  if (collision_check_enable_) {
    auto collision_reason = robot_->get_predicted_collision_reason(sdk_target_positions, collision_threshold_);
    if (collision_reason.has_value()) {
      auto & clock = *rclcpp::Clock::make_shared();
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RBY1SystemHardware"), clock, 1000,
                           "[PREDICTIVE COLLISION REJECTED] %s. Holding current position.",
                           collision_reason->c_str());
      
      // Safety: overwrite commanded targets with current actual positions to force a stop
      if (sdk_positions.size() == (size_t)dof) {
        sdk_target_positions = sdk_positions;
      }
    }
  }

  // Send stream command
  robot_->send_stream_command(sdk_target_positions);

  return hardware_interface::return_type::OK;
}

} // namespace rby1_hardware

PLUGINLIB_EXPORT_CLASS(rby1_hardware::RBY1SystemHardware, hardware_interface::SystemInterface)
