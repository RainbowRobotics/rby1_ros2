#!/usr/bin/env python3
"""
Stream Command Example (Cartesian and Joint Overhaul)
======================================================
Demonstrates persistent command stream longevity and responsiveness by enabling
persistent stream mode and then sending:
  1. Step 1: Interpolated Joint commands from Zero to Ready pose.
  2. Step 2: Interpolated Cartesian commands to move arms Z-up by 5cm.
  3. Step 3: Interpolated Cartesian commands with impedance to move arms Z-down by 5cm.
  4. Step 4: Interpolated Joint commands back to Zero pose.

Run:
  ros2 run rby1_examples 13_stream_command
"""
import time
import copy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rby1_msgs.action import Rby1JointCommand, StreamJoint, StreamCartesian
from rby1_msgs.msg import JointCommand, RobotState, StreamJointCommand, StreamCartesianCommand, CartesianCommand
from rby1_msgs.srv import StateOnOff, GetCartesianPose
from geometry_msgs.msg import Transform

class StreamCommand(Node):
    def __init__(self):
        super().__init__('stream_command')
        self.stream_hz = 15.0
        self._pub = ActionClient(self, StreamJoint, 'stream_joint')
        self._cartesian_pub = ActionClient(self, StreamCartesian, 'stream_cartesian')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.stream_control_client = self.create_client(StateOnOff, 'stream_control')
        self.get_pose_client = self.create_client(GetCartesianPose, 'get_cartesian_pose')
        self._zero_pose_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        
        self.state_sub = self.create_subscription(RobotState, 'robot_state', self.state_callback, 10)
        self.control_state = None

        # Build joint ready pose values
        self.zero_torso = [0.0] * 6
        self.zero_right = [0.0] * 7
        self.zero_left  = [0.0] * 7
        self.zero_head  = [0.0] * 2

        self.ready_torso = [0.0] * 6
        self.ready_right = [0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0]
        self.ready_left  = [0.0, 0.5, 0.0, -1.0, 0.0, 0.0, 0.0]
        self.ready_head  = [0.0, 0.0]

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

    def spin_sleep(self, duration: float):
        start_time = self.get_clock().now()
        while rclpy.ok():
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed >= duration:
                break
            rclpy.spin_once(self, timeout_sec=0.01)

    def ensure_robot_ready(self):
        self.get_logger().info('Ensuring robot is powered on and servos are active...')
        rclpy.spin_once(self, timeout_sec=0.5)
        if self.control_state in [2, 3]:
            self.get_logger().info('Robot is already enabled.')
            return True

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
        
        self.spin_sleep(2.0)
            
        self.get_logger().info('Sending Servo ON request...')
        self.servo_client.wait_for_service()
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to servo on: {future.result().message}')
            return False
            
        if self.wait_for_state([2, 3], timeout=15.0):
            self.get_logger().info('Robot is ready.')
            self.spin_sleep(1.0)
            return True
        else:
            self.get_logger().error(f'Timed out waiting for robot to enable.')
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

    def toggle_stream(self, enable: bool, value: float = 0.0, mode: str = "normal") -> bool:
        if not rclpy.ok():
            return False
        try:
            req = StateOnOff.Request()
            req.state = enable
            req.parameters = mode
            req.value = value
            self.get_logger().info(f"Calling stream_control: state={enable}, value={value}...")
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
            print(f"Exception during stream control toggle: {e}")
        return False

    def get_cartesian_pose(self, ref_link, target_link):
        req = GetCartesianPose.Request()
        req.ref_link = ref_link
        req.target_link = target_link
        self.get_pose_client.wait_for_service()
        future = self.get_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().transform

    def send_joint_trajectory(self, start_torso, target_torso,
                              start_right, target_right,
                              start_left, target_left,
                              start_head, target_head,
                              duration: float):
        num_points = int(duration * self.stream_hz)
        dt = 1.0 / self.stream_hz
        minimum_time = dt * 1.5

        self._pub.wait_for_server()

        for i in range(1, num_points + 1):
            ratio = i / num_points
            
            curr_torso = [s + (t - s) * ratio for s, t in zip(start_torso, target_torso)]
            curr_right = [s + (t - s) * ratio for s, t in zip(start_right, target_right)]
            curr_left = [s + (t - s) * ratio for s, t in zip(start_left, target_left)]
            curr_head = [s + (t - s) * ratio for s, t in zip(start_head, target_head)]
            
            goal_msg = StreamJoint.Goal()
            cmd = StreamJointCommand()
            
            cmd.torso = JointCommand()
            cmd.torso.position = curr_torso
            cmd.torso.minimum_time = minimum_time
            
            cmd.right_arm = JointCommand()
            cmd.right_arm.position = curr_right
            cmd.right_arm.minimum_time = minimum_time
            
            cmd.left_arm = JointCommand()
            cmd.left_arm.position = curr_left
            cmd.left_arm.minimum_time = minimum_time
            
            cmd.head = JointCommand()
            cmd.head.position = curr_head
            cmd.head.minimum_time = minimum_time
            
            goal_msg.command = cmd
            self._pub.send_goal_async(goal_msg)
            self.spin_sleep(dt)

    def send_cartesian_trajectory(self, start_right, target_right,
                                  start_left, target_left,
                                  duration: float,
                                  use_impedance: bool = False):
        num_points = int(duration * self.stream_hz)
        dt = 1.0 / self.stream_hz
        minimum_time = dt * 1.5

        self._cartesian_pub.wait_for_server()

        for i in range(1, num_points + 1):
            ratio = i / num_points
            
            # Interpolate translation for right arm
            curr_right = Transform()
            curr_right.translation.x = start_right.translation.x + (target_right.translation.x - start_right.translation.x) * ratio
            curr_right.translation.y = start_right.translation.y + (target_right.translation.y - start_right.translation.y) * ratio
            curr_right.translation.z = start_right.translation.z + (target_right.translation.z - start_right.translation.z) * ratio
            curr_right.rotation.x = start_right.rotation.x
            curr_right.rotation.y = start_right.rotation.y
            curr_right.rotation.z = start_right.rotation.z
            curr_right.rotation.w = start_right.rotation.w
            
            # Interpolate translation for left arm
            curr_left = Transform()
            curr_left.translation.x = start_left.translation.x + (target_left.translation.x - start_left.translation.x) * ratio
            curr_left.translation.y = start_left.translation.y + (target_left.translation.y - start_left.translation.y) * ratio
            curr_left.translation.z = start_left.translation.z + (target_left.translation.z - start_left.translation.z) * ratio
            curr_left.rotation.x = start_left.rotation.x
            curr_left.rotation.y = start_left.rotation.y
            curr_left.rotation.z = start_left.rotation.z
            curr_left.rotation.w = start_left.rotation.w

            if i == 1:
                self.get_logger().info(f"First Cartesian Target Right Rotation: "
                                       f"x={curr_right.rotation.x:.4f}, "
                                       f"y={curr_right.rotation.y:.4f}, "
                                       f"z={curr_right.rotation.z:.4f}, "
                                       f"w={curr_right.rotation.w:.4f}")

            goal_msg = StreamCartesian.Goal()
            cmd = StreamCartesianCommand()
            
            cmd.right_arm = CartesianCommand()
            cmd.right_arm.ref_link = "link_torso_5"
            cmd.right_arm.target_link = "link_right_arm_6"
            cmd.right_arm.transform = curr_right
            cmd.right_arm.use_impedance = use_impedance
            cmd.right_arm.minimum_time = minimum_time
            if use_impedance:
                cmd.right_arm.translation_weight = [500.0, 500.0, 500.0]
                cmd.right_arm.rotation_weight = [50.0, 50.0, 50.0]
                cmd.right_arm.control_hold_time = 5.0
                
            cmd.left_arm = CartesianCommand()
            cmd.left_arm.ref_link = "link_torso_5"
            cmd.left_arm.target_link = "link_left_arm_6"
            cmd.left_arm.transform = curr_left
            cmd.left_arm.use_impedance = use_impedance
            cmd.left_arm.minimum_time = minimum_time
            if use_impedance:
                cmd.left_arm.translation_weight = [500.0, 500.0, 500.0]
                cmd.left_arm.rotation_weight = [50.0, 50.0, 50.0]
                cmd.left_arm.control_hold_time = 5.0
                
            goal_msg.command = cmd
            self._cartesian_pub.send_goal_async(goal_msg)
            self.spin_sleep(dt)

def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = StreamCommand()

    # 1. Prepare Robot
    if not node.ensure_robot_ready():
        node.get_logger().error('Failed to prepare robot. Exiting.')
        return

    # Go to zero pose first to make sure robot starts from [0.0]*22
    if not node.go_to_zero_pose():
        node.get_logger().error('Failed to establish zero pose.')
        return

    # 2. Enable Persistent Command Stream
    if not node.toggle_stream(True, value=node.stream_hz):
        node.get_logger().error('Failed to enable command stream. Exiting.')
        return

    try:
        # Step 1: Zero Pose -> Ready Pose (Joint command, interpolated over 5.0 seconds)
        node.get_logger().info('\n==========================================')
        node.get_logger().info(' Step 1: Moving to Ready Pose (Interpolated Joint Space)')
        node.get_logger().info('==========================================')
        node.send_joint_trajectory(
            node.zero_torso, node.ready_torso,
            node.zero_right, node.ready_right,
            node.zero_left, node.ready_left,
            node.zero_head, node.ready_head,
            duration=5.0
        )
        node.get_logger().info('Step 1 complete. Waiting for 1.0 second...')
        node.spin_sleep(1.0)

        # Query current Cartesian poses
        node.get_logger().info('Querying current Cartesian poses...')
        ready_right_trans = node.get_cartesian_pose("link_torso_5", "link_right_arm_6")
        ready_left_trans = node.get_cartesian_pose("link_torso_5", "link_left_arm_6")
        node.get_logger().info(f"ready_right_trans: {ready_right_trans}")
        node.get_logger().info(f"ready_left_trans: {ready_left_trans}")

        # Define Z-up target (Z + 0.05m)
        up_right_trans = copy.deepcopy(ready_right_trans)
        up_right_trans.translation.z += 0.05
        
        up_left_trans = copy.deepcopy(ready_left_trans)
        up_left_trans.translation.z += 0.05

        # Step 2: Ready Pose -> Move Z axis UP (Cartesian position control, over 5.0 seconds)
        node.get_logger().info('\n==========================================')
        node.get_logger().info(' Step 2: Moving Z axis UP (Cartesian Position Control)')
        node.get_logger().info('==========================================')
        node.send_cartesian_trajectory(
            ready_right_trans, up_right_trans,
            ready_left_trans, up_left_trans,
            duration=5.0,
            use_impedance=False
        )
        node.get_logger().info('Step 2 complete. Waiting for 1.0 second...')
        node.spin_sleep(1.0)

        # Step 3: Move Z axis BACK DOWN (Cartesian impedance control, over 5.0 seconds)
        node.get_logger().info('\n==========================================')
        node.get_logger().info(' Step 3: Moving Z axis DOWN (Cartesian Impedance Control)')
        node.get_logger().info('==========================================')
        node.send_cartesian_trajectory(
            up_right_trans, ready_right_trans,
            up_left_trans, ready_left_trans,
            duration=5.0,
            use_impedance=True
        )
        node.get_logger().info('Step 3 complete. Waiting for 1.0 second...')
        node.spin_sleep(1.0)

        # Step 4: Ready Pose -> Zero Pose (Joint command, interpolated over 5.0 seconds)
        node.get_logger().info('\n==========================================')
        node.get_logger().info(' Step 4: Returning to Zero Pose (Interpolated Joint Space)')
        node.get_logger().info('==========================================')
        node.send_joint_trajectory(
            node.ready_torso, node.zero_torso,
            node.ready_right, node.zero_right,
            node.ready_left, node.zero_left,
            node.ready_head, node.zero_head,
            duration=5.0
        )
        node.get_logger().info('Step 4 complete.')

    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down...')
    finally:
        # 4. Safely deactivate stream on exit
        if rclpy.ok():
            node.get_logger().info('Cleaning up: Deactivating stream control...')
            node.toggle_stream(False)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
