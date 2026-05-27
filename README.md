# RBY1 ROS 2 Control Driver

This branch is a clean ROS 2 Humble implementation of the RBY1 driver foundation using `ros2_control`.

The first implementation is intentionally state-first and conservative:

- `hardware_interface::SystemInterface` exposes joint position, velocity, and effort state interfaces.
- `joint_state_broadcaster` publishes standard `/joint_states`.
- A separate robot manager node provides non-real-time management services for power, servo, brakes, tool flange power, gravity compensation, and control-manager commands.
- Motion command interfaces are not exported in this version, so no controller can move the robot through `ros2_control` yet.

## Packages

- `rby1_msgs`: Stable status messages and management services.
- `rby1_description`: Xacro descriptions for RBY1-A and RBY1-M with `ros2_control` tags.
- `rby1_hardware_interface`: RBY1 SDK adapter and `ros2_control` system plugin.
- `rby1_robot_manager`: Non-real-time service and status-topic node.
- `rby1_bringup`: Launch and controller configuration.

## Build

Set the RBY1 SDK path before building real hardware support:

```bash
export RBY1_SDK_PATH=/path/to/rby1-sdk
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Source the workspace after building:

```bash
source install/setup.bash
```

## Mock Hardware

Use mock hardware to validate the ROS graph without connecting to a robot:

```bash
ros2 launch rby1_bringup rby1_mock.launch.py model:=a
```

## Real Hardware Read-Only Bringup

```bash
ros2 launch rby1_bringup rby1_control.launch.py robot_ip:=192.168.0.10:50051 model:=a
```

The hardware interface only reads state in this version. Power, servo, brake, and control-manager changes are explicit service calls through `rby1_robot_manager`.

## Services

When `launch_robot_manager:=true`, the manager node provides:

- `/rby1/robot_power`
- `/rby1/robot_servo`
- `/rby1/set_motor_brake`
- `/rby1/tool_flange_power`
- `/rby1/gravity_compensation`
- `/rby1/control_manager_command`

## Status Topics

- `/joint_states` from `joint_state_broadcaster`
- `/rby1/robot_state`
- `/rby1/battery_state` when enabled
- `/rby1/tool_flange/left` and `/rby1/tool_flange/right` when enabled

## Current Scope

This branch deliberately does not include the old monolithic `rby1_driver` node, action servers, Cartesian commands, streaming trajectory commands, or MoveIt integration. Those features should be added incrementally after the read-only hardware interface and management layer are stable on real hardware.
