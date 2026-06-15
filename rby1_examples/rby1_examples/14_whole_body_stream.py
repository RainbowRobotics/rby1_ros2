#!/usr/bin/env python3
"""
Whole-Body Stream Command Example
==================================
Demonstrates whole-body stream control. Activates whole-body stream mode,
sends joint commands via StreamJoint action in a loop, and concurrently
publishes base velocity commands to cmd_vel.

Run:
  ros2 run rby1_examples 14_whole_body_stream
"""
import time
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rby1_msgs.action import Rby1JointCommand, StreamJoint
from rby1_msgs.msg import JointCommand, RobotState, StreamJointCommand
from rby1_msgs.srv import StateOnOff
from geometry_msgs.msg import Twist

class WholeBodyStreamCommand(Node):
    def __init__(self):
        super().__init__('whole_body_stream_command')
        self.stream_hz = 30.0
        
        # ROS 2 publishers, clients and action clients
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        self.stream_joint_client = ActionClient(self, StreamJoint, 'stream_joint')
        
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.stream_control_client = self.create_client(StateOnOff, 'stream_control')
        
        self.state_sub = self.create_subscription(RobotState, 'robot_state', self.state_callback, 10)
        self.control_state = None

        # Build joint ready/zero pose values
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

        self.joint_client.wait_for_server()
        future = self.joint_client.send_goal_async(goal_msg)
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

    def toggle_stream(self, enable: bool, mode: str = "whole_body") -> bool:
        if not rclpy.ok():
            return False
        try:
            req = StateOnOff.Request()
            req.state = enable
            req.parameters = mode
            self.get_logger().info(f"Calling stream_control: state={enable}, mode={mode}...")
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

def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WholeBodyStreamCommand()

    # 1. Prepare Robot
    if not node.ensure_robot_ready():
        node.get_logger().error('Failed to prepare robot. Exiting.')
        return

    # Go to zero pose first to make sure robot starts from [0.0]*22
    if not node.go_to_zero_pose():
        node.get_logger().error('Failed to establish zero pose.')
        return

    # 2. Enable Persistent Command Stream in whole_body mode
    if not node.toggle_stream(True, mode="whole_body"):
        node.get_logger().error('Failed to enable whole-body stream. Exiting.')
        return

    node.stream_joint_client.wait_for_server()

    try:
        # Step 1: Zero Pose -> Ready Pose (Stream Joint Command in loop over 3.0 seconds)
        node.get_logger().info('\n==========================================')
        node.get_logger().info(' Step 1: Moving to Ready Pose via Stream (3.0s)')
        node.get_logger().info('==========================================')
        
        duration = 3.0
        num_points = int(duration * node.stream_hz)
        dt = 1.0 / node.stream_hz

        for i in range(1, num_points + 1):
            ratio = i / num_points
            
            curr_torso = [s + (t - s) * ratio for s, t in zip(node.zero_torso, node.ready_torso)]
            curr_right = [s + (t - s) * ratio for s, t in zip(node.zero_right, node.ready_right)]
            curr_left = [s + (t - s) * ratio for s, t in zip(node.zero_left, node.ready_left)]
            curr_head = [s + (t - s) * ratio for s, t in zip(node.zero_head, node.ready_head)]
            
            goal_msg = StreamJoint.Goal()
            cmd = StreamJointCommand()
            
            cmd.torso = JointCommand()
            cmd.torso.position = curr_torso
            
            cmd.right_arm = JointCommand()
            cmd.right_arm.position = curr_right
            
            cmd.left_arm = JointCommand()
            cmd.left_arm.position = curr_left
            
            cmd.head = JointCommand()
            cmd.head.position = curr_head
            
            goal_msg.command = cmd
            node.stream_joint_client.send_goal_async(goal_msg)
            node.spin_sleep(dt)

        node.get_logger().info('Step 1 complete. Waiting 1.0s...')
        node.spin_sleep(1.0)

        # Step 2: Return upper body to Zero Pose (over 3s) while mobile base does 2 round trips (8s total)
        node.get_logger().info('\n==========================================')
        node.get_logger().info(' Step 2: Returning to Zero Pose & Mobile Base Round Trips (8.0s)')
        node.get_logger().info('==========================================')

        total_duration = 8.0
        total_points = int(total_duration * node.stream_hz)

        for i in range(1, total_points + 1):
            t = i * dt
            
            # Upper body joint command calculation
            if t <= 3.0:
                # Return to Zero Pose interpolation (over 3.0s)
                ratio = t / 3.0
                curr_torso = [s + (t_val - s) * ratio for s, t_val in zip(node.ready_torso, node.zero_torso)]
                curr_right = [s + (t_val - s) * ratio for s, t_val in zip(node.ready_right, node.zero_right)]
                curr_left = [s + (t_val - s) * ratio for s, t_val in zip(node.ready_left, node.zero_left)]
                curr_head = [s + (t_val - s) * ratio for s, t_val in zip(node.ready_head, node.zero_head)]
            else:
                # Keep holding at Zero Pose
                curr_torso = node.zero_torso
                curr_right = node.zero_right
                curr_left = node.zero_left
                curr_head = node.zero_head

            # Mobile base command calculation (relative_t in 4-second cycles)
            relative_t = t % 4.0
            twist_msg = Twist()
            if relative_t < 1.0:
                twist_msg.linear.x = 0.15  # Forward
            elif 1.0 <= relative_t < 2.0:
                twist_msg.linear.x = 0.0   # Stop
            elif 2.0 <= relative_t < 3.0:
                twist_msg.linear.x = -0.15 # Backward
            else:
                twist_msg.linear.x = 0.0   # Stop

            # Publish base command
            node.cmd_vel_pub.publish(twist_msg)

            # Send joint stream command
            goal_msg = StreamJoint.Goal()
            cmd = StreamJointCommand()
            
            cmd.torso = JointCommand()
            cmd.torso.position = curr_torso
            
            cmd.right_arm = JointCommand()
            cmd.right_arm.position = curr_right
            
            cmd.left_arm = JointCommand()
            cmd.left_arm.position = curr_left
            
            cmd.head = JointCommand()
            cmd.head.position = curr_head
            
            goal_msg.command = cmd
            node.stream_joint_client.send_goal_async(goal_msg)
            node.spin_sleep(dt)

        # Stop mobile base explicitly
        node.get_logger().info('Mobile base round trips finished. Stopping base.')
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)

        # Step 3: Hold Zero Pose and stay still for 3.0 seconds
        node.get_logger().info('\n==========================================')
        node.get_logger().info(' Step 3: Holding Zero Pose (3.0s)')
        node.get_logger().info('==========================================')

        hold_duration = 3.0
        hold_points = int(hold_duration * node.stream_hz)
        for i in range(1, hold_points + 1):
            goal_msg = StreamJoint.Goal()
            cmd = StreamJointCommand()
            
            cmd.torso = JointCommand()
            cmd.torso.position = node.zero_torso
            
            cmd.right_arm = JointCommand()
            cmd.right_arm.position = node.zero_right
            
            cmd.left_arm = JointCommand()
            cmd.left_arm.position = node.zero_left
            
            cmd.head = JointCommand()
            cmd.head.position = node.zero_head
            
            goal_msg.command = cmd
            node.stream_joint_client.send_goal_async(goal_msg)
            node.spin_sleep(dt)

        node.get_logger().info('Finished whole-body stream execution.')

    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down...')
    finally:
        # Deactivate stream on exit
        if rclpy.ok():
            node.get_logger().info('Cleaning up: Deactivating stream control...')
            node.toggle_stream(False)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
