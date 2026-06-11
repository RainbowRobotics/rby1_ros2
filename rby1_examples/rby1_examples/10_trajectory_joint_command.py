#!/usr/bin/env python3
"""
Trajectory Joint Command Example
===============================
Demonstrates multi-point whole-body trajectory streaming via the standard FollowJointTrajectory
action client over a persistent command stream.

Sequence (Phase 1 — Position Control):
  1. Ensure the robot is powered and enabled.
  2. Move whole body to Zero Pose.
  3. Enable a persistent command stream via '/stream_control'.
  4. Send a whole-body trajectory (Zero → Target) using FollowJointTrajectory.
  5. Disable stream, return to Zero Pose.

Sequence (Phase 2 — Impedance Control):
  6. Call '/set_trajectory_impedance' with state=True to switch to impedance mode.
  7. Enable a persistent command stream via '/stream_control'.
  8. Send the same whole-body trajectory again (Zero → Target) in impedance mode.
  9. Disable stream, disable impedance, return to Zero Pose.

NOTE: 'set_trajectory_impedance' applies only to follow_joint_trajectory.
      It does not affect robot_joint action commands.
      Head does not support impedance — the driver falls back to position control for head joints.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from rby1_msgs.action import Rby1JointCommand
from rby1_msgs.msg import RobotState, JointCommand
from rby1_msgs.srv import StateOnOff, SetTrajectoryImpedance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryJointCommand(Node):
    def __init__(self):
        super().__init__('trajectory_joint_command')
        self._stream_client = ActionClient(self, FollowJointTrajectory, 'follow_joint_trajectory')
        self._zero_pose_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.stream_control_client = self.create_client(StateOnOff, 'stream_control')
        self.impedance_client = self.create_client(SetTrajectoryImpedance, 'set_trajectory_impedance')

        self.state_sub = self.create_subscription(RobotState, 'robot_state', self.state_callback, 10)
        self.control_state = None

    def state_callback(self, msg):
        self.control_state = msg.control_manager_state

    def wait_for_state(self, target_states, timeout=5.0):
        start_time = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.control_state in target_states:
                return True
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                return False
        return False

    def ensure_robot_ready(self):
        self.get_logger().info('Ensuring robot is powered on and servos are active...')

        # 1. Check current state first
        rclpy.spin_once(self, timeout_sec=0.5)
        if self.control_state in [2, 3]:
            self.get_logger().info('Robot is already enabled.')
            return True

        # 2. Power On
        req = StateOnOff.Request()
        req.state = True
        req.parameters = "all"
        self.get_logger().info('Sending Power ON request...')
        self.power_client.wait_for_service()
        future = self.power_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to power on: {future.result().message}')
            return False

        time.sleep(1.0)  # Wait for power to stabilize

        # 3. Servo On
        self.get_logger().info('Sending Servo ON request...')
        self.servo_client.wait_for_service()
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to servo on: {future.result().message}')
            return False

        # 4. Wait for state to become 2 or 3
        if self.wait_for_state([2, 3], timeout=10.0):
            self.get_logger().info('Robot is ready.')
            time.sleep(1.0)
            return True
        else:
            self.get_logger().error(f'Timed out waiting for robot to enable. Current state: {self.control_state}')
            return False

    def toggle_stream(self, enable: bool) -> bool:
        if not rclpy.ok():
            return False
        try:
            req = StateOnOff.Request()
            req.state = enable
            req.parameters = ""
            self.get_logger().info(f"Calling stream_control: state={enable}...")
            self.stream_control_client.wait_for_service(timeout_sec=1.0)
            future = self.stream_control_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done():
                res = future.result()
                if res and res.success:
                    self.get_logger().info(f"Stream Control successfully {'enabled' if enable else 'disabled'}.")
                    return True
                else:
                    self.get_logger().error(f"Failed to toggle stream control: {res.message if res else 'No response'}")
            else:
                self.get_logger().error("Timeout toggling stream control.")
        except Exception as e:
            self.get_logger().error(f"Exception during stream control toggle: {e}")
        return False

    def set_impedance(self, enable: bool,
                      stiffness: list = None,
                      damping_ratio: float = 1.0,
                      torque_limit: float = 10.0) -> bool:
        """Enable or disable impedance mode for follow_joint_trajectory.

        Args:
            enable: True to activate impedance control, False to revert to position control.
            stiffness: Joint stiffness list [N·m/rad], ordered torso(6)+right_arm(7)+left_arm(7)+head(2)=22.
                       Leave None or empty list to use the default value of 100.0 for all joints.
            damping_ratio: Dimensionless damping ratio (default 1.0).
            torque_limit: Per-joint torque limit in N·m (default 10.0).
        """
        req = SetTrajectoryImpedance.Request()
        req.state = enable
        req.stiffness = stiffness if stiffness else []
        req.damping_ratio = damping_ratio
        req.torque_limit = torque_limit

        self.get_logger().info(
            f"Calling set_trajectory_impedance: state={enable}, "
            f"stiffness={'default(100.0)' if not stiffness else f'{len(stiffness)} values'}, "
            f"damping_ratio={damping_ratio}, torque_limit={torque_limit}"
        )
        self.impedance_client.wait_for_service(timeout_sec=2.0)
        future = self.impedance_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.done():
            res = future.result()
            if res and res.success:
                self.get_logger().info(f"set_trajectory_impedance OK: {res.message}")
                return True
            else:
                self.get_logger().error(f"set_trajectory_impedance FAILED: {res.message if res else 'No response'}")
        else:
            self.get_logger().error("Timeout calling set_trajectory_impedance.")
        return False

    def go_to_zero_pose(self):
        self.get_logger().info('Moving Whole Body to Zero Pose...')
        goal_msg = Rby1JointCommand.Goal()

        for part in ['torso', 'right_arm', 'left_arm', 'head']:
            cmd = JointCommand()
            if part == 'torso':
                cmd.position = [0.0] * 6
            elif part == 'head':
                cmd.position = [0.0] * 2
            elif part in ['right_arm', 'left_arm']:
                cmd.position = [0.0] * 7
            cmd.minimum_time = 3.0
            setattr(goal_msg, part, cmd)

        self._zero_pose_client.wait_for_server()
        future = self._zero_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle.accepted:
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            result = res_future.result().result
            if result.success:
                self.get_logger().info('Zero Pose reached successfully.')
                return True
            else:
                self.get_logger().error(f'Zero Pose failed with code: {result.finish_code}')
                return False
        return True

    def send_stream_goal(self, trajectory):
        self.get_logger().info('Starting Trajectory Streaming...')
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self._stream_client.wait_for_server()
        return self._stream_client.send_goal_async(goal_msg)


def build_trajectory(joint_names, full_start, full_target, num_points=10, total_sec=5.0):
    """Build a linear interpolation JointTrajectory from start to target."""
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    for i in range(1, num_points + 1):
        point = JointTrajectoryPoint()
        point.positions = [(s + (t - s) * i / num_points) for s, t in zip(full_start, full_target)]
        elapsed_ms = int(total_sec * 1000 * i / num_points)
        point.time_from_start.sec = elapsed_ms // 1000
        point.time_from_start.nanosec = (elapsed_ms % 1000) * 1_000_000
        trajectory.points.append(point)
    return trajectory


def run_trajectory_phase(action_client, label, trajectory):
    """Enable stream → send trajectory → wait for result → disable stream."""
    action_client.get_logger().info(f'[{label}] Enabling persistent stream...')
    if not action_client.toggle_stream(True):
        action_client.get_logger().error(f'[{label}] Failed to enable stream. Skipping phase.')
        return False

    time.sleep(0.5)

    action_client.get_logger().info(f'[{label}] Sending trajectory...')
    future = action_client.send_stream_goal(trajectory)
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()

    if goal_handle.accepted:
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(action_client, get_result_future)
        result = get_result_future.result().result
        action_client.get_logger().info(
            f'[{label}] Done: error_code={result.error_code}, msg="{result.error_string}"')

    time.sleep(2.0)

    action_client.get_logger().info(f'[{label}] Disabling stream...')
    action_client.toggle_stream(False)
    time.sleep(0.5)
    return True


def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryJointCommand()

    # --- Common trajectory setup ---
    joint_names = [f'torso_{i}' for i in range(6)] + \
                  [f'right_arm_{i}' for i in range(7)] + \
                  [f'left_arm_{i}' for i in range(7)] + \
                  [f'head_{i}' for i in range(2)]

    target_torso = [0.0, 0.1, -0.2, 0.1, 0.0, 0.0]
    target_right = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    target_left  = [0.0,  0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    target_head  = [0.0, 0.0]

    full_target = target_torso + target_right + target_left + target_head
    full_start  = [0.0] * len(full_target)

    # 0. Ensure Robot is Power ON and Servo ON
    if not action_client.ensure_robot_ready():
        action_client.get_logger().error('Failed to prepare robot. Exiting.')
        return

    # =========================================================
    # Phase 1: Position Control  (Zero → Target → Zero)
    # =========================================================
    action_client.get_logger().info('=' * 50)
    action_client.get_logger().info('Phase 1: Position Control (Zero → Target → Zero)')
    action_client.get_logger().info('=' * 50)

    if not action_client.go_to_zero_pose():
        action_client.get_logger().error('Failed to establish zero pose.')
        return

    traj_pos = build_trajectory(joint_names, full_start, full_target)
    run_trajectory_phase(action_client, 'PositionCtrl', traj_pos)

    action_client.get_logger().info('[Phase 1] Returning to Zero Pose...')
    action_client.go_to_zero_pose()
    time.sleep(1.5)

    # =========================================================
    # Phase 2: Impedance Control  (Impedance ON → Target → Zero)
    # =========================================================
    action_client.get_logger().info('=' * 50)
    action_client.get_logger().info('Phase 2: Impedance Control (Impedance ON → Target → Zero)')
    action_client.get_logger().info('=' * 50)

    # Stiffness order: torso(6) + right_arm(7) + left_arm(7) + head(2) = 22 joints total.
    # Head is always position-controlled by the driver regardless of this value.
    stiffness = (
        [200.0] * 6 +  # torso: stiffer
        [100.0] * 7 +  # right arm: compliant
        [100.0] * 7 +  # left arm: compliant
        [50.0]  * 2    # head: driver ignores, falls back to position ctrl
    )
    if not action_client.set_impedance(True, stiffness=stiffness, damping_ratio=1.0, torque_limit=10.0):
        action_client.get_logger().error('[Phase 2] Failed to enable impedance mode. Skipping.')
    else:
        traj_imp = build_trajectory(joint_names, full_start, full_target)
        run_trajectory_phase(action_client, 'ImpedanceCtrl', traj_imp)

        # Disable impedance before returning to zero (zero uses robot_joint, not follow_joint_trajectory,
        # but disabling here keeps the state clean)
        action_client.set_impedance(False)
        action_client.get_logger().info('[Phase 2] Returning to Zero Pose...')
        action_client.go_to_zero_pose()

    action_client.get_logger().info('Example complete.')
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
