#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rby1_hardware_interface/rby1_robot_client.hpp"

namespace rby1_hardware_interface
{

class Rby1SystemInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool validate_joint_interfaces(const hardware_interface::HardwareInfo & hardware_info) const;
  bool apply_state_snapshot(const RobotStateSnapshot & snapshot);
  std::string hardware_parameter_or(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & key,
    const std::string & fallback) const;

  rclcpp::Logger logger_{rclcpp::get_logger("rby1_system_interface")};
  RobotClientOptions options_;
  std::unique_ptr<Rby1RobotClient> robot_client_;
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> joint_index_by_name_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;
  bool active_{false};
  size_t read_error_count_{0};
};

}  // namespace rby1_hardware_interface
