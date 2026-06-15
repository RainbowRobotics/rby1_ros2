#!/usr/bin/env python3
"""
Stream Command Example (Normal Stream Mode)
===========================================
Demonstrates normal stream mode behavior. Updates arms joint positions,
queries and moves end-effector Cartesian poses, demonstrates goal preemption,
and safe return to Zero Pose.

Run:
  ros2 run rby1_examples 13_stream_command
"""
import time
import copy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rby1_msgs.action import Rby1CartesianCommand, Rby1JointCommand
from rby1_msgs.msg import JointCommand, RobotState, CartesianCommand
from rby1_msgs.srv import StateOnOff, GetCartesianPose

class StreamCommand(Node):
    def __init__(self):
        super().__init__('stream_command')
        self.stream_hz = 15.0
        self.joint_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        self.cartesian_client = ActionClient(self, Rby1CartesianCommand, 'robot_cartesian')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.stream_control_client = self.create_client(StateOnOff, 'stream_control')
        self.get_pose_client = self.create_client(GetCartesianPose, 'get_cartesian_pose')
        
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

    def toggle_stream(self, enable: bool, mode: str = "normal") -> bool:
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

    def get_cartesian_pose(self, ref_link, target_link):
        req = GetCartesianPose.Request()
        req.ref_link = ref_link
        req.target_link = target_link
        self.get_pose_client.wait_for_service()
        future = self.get_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().transform

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

    # 2. Enable Persistent Command Stream in normal mode
    if not node.toggle_stream(True, mode="normal"):
        node.get_logger().error('Failed to enable command stream. Exiting.')
        return

    try:
        # Step 1: Move both arms to Ready Pose using Joint control
        node.get_logger().info('\n==========================================')
        node.get_logger().info(' Step 1: Moving Arms to Ready Pose (Joint Command, 3.0s)')
        node.get_logger().info('==========================================')
        
        goal_msg = Rby1JointCommand.Goal()
        # Right Arm Ready
        goal_msg.right_arm = JointCommand()
        goal_msg.right_arm.position = [0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0]
        goal_msg.right_arm.minimum_time = 3.0
        # Left Arm Ready
        goal_msg.left_arm = JointCommand()
        goal_msg.left_arm.position = [0.0, 0.5, 0.0, -1.0, 0.0, 0.0, 0.0]
        goal_msg.left_arm.minimum_time = 3.0
        goal_msg.priority = 10
        
        node.get_logger().info('Sending Joint Goal...')
        node.joint_client.wait_for_server()
        future = node.joint_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            node.get_logger().error('Joint command rejected.')
            return
        
        get_res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, get_res_future)
        node.get_logger().info('Ready Pose reached.')
        node.spin_sleep(1.0)

        # Step 2: Move Z-axis UP using Cartesian control (refer to example 8)
        node.get_logger().info('\n==========================================')
        node.get_logger().info(' Step 2: Moving Z-axis UP (Cartesian Command, 3.0s)')
        node.get_logger().info('==========================================')
        
        # Query current Cartesian poses
        ready_right_trans = node.get_cartesian_pose("link_torso_5", "link_right_arm_6")
        ready_left_trans = node.get_cartesian_pose("link_torso_5", "link_left_arm_6")
        node.get_logger().info(f"ready_right_trans: {ready_right_trans}")
        node.get_logger().info(f"ready_left_trans: {ready_left_trans}")

        # Define Z-up target (Z + 0.05m)
        up_right_trans = copy.deepcopy(ready_right_trans)
        up_right_trans.translation.z += 0.05
        up_left_trans = copy.deepcopy(ready_left_trans)
        up_left_trans.translation.z += 0.05

        cart_goal_msg = Rby1CartesianCommand.Goal()
        # Right Arm UP
        cart_goal_msg.right_arm = CartesianCommand()
        cart_goal_msg.right_arm.ref_link = "link_torso_5"
        cart_goal_msg.right_arm.target_link = "link_right_arm_6"
        cart_goal_msg.right_arm.transform = up_right_trans
        cart_goal_msg.right_arm.minimum_time = 3.0
        
        # Left Arm UP
        cart_goal_msg.left_arm = CartesianCommand()
        cart_goal_msg.left_arm.ref_link = "link_torso_5"
        cart_goal_msg.left_arm.target_link = "link_left_arm_6"
        cart_goal_msg.left_arm.transform = up_left_trans
        cart_goal_msg.left_arm.minimum_time = 3.0
        
        node.get_logger().info('Sending Cartesian Goal (UP)...')
        node.cartesian_client.wait_for_server()
        future = node.cartesian_client.send_goal_async(cart_goal_msg)
        rclpy.spin_until_future_complete(node, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            node.get_logger().error('Cartesian command (UP) rejected.')
            return
        
        get_res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, get_res_future)
        node.get_logger().info('Cartesian UP complete.')
        node.spin_sleep(1.0)

        # Step 3: Move Z-axis BACK DOWN to Ready Pose, then preempt and return to Zero Pose
        node.get_logger().info('\n==========================================')
        node.get_logger().info(' Step 3: Moving Z-axis DOWN and Preempting to Zero Pose')
        node.get_logger().info('==========================================')
        
        cart_goal_msg_down = Rby1CartesianCommand.Goal()
        # Right Arm DOWN to ready_right_trans
        cart_goal_msg_down.right_arm = CartesianCommand()
        cart_goal_msg_down.right_arm.ref_link = "link_torso_5"
        cart_goal_msg_down.right_arm.target_link = "link_right_arm_6"
        cart_goal_msg_down.right_arm.transform = ready_right_trans
        cart_goal_msg_down.right_arm.minimum_time = 5.0
        
        # Left Arm DOWN to ready_left_trans
        cart_goal_msg_down.left_arm = CartesianCommand()
        cart_goal_msg_down.left_arm.ref_link = "link_torso_5"
        cart_goal_msg_down.left_arm.target_link = "link_left_arm_6"
        cart_goal_msg_down.left_arm.transform = ready_left_trans
        cart_goal_msg_down.left_arm.minimum_time = 5.0

        node.get_logger().info('Sending Cartesian Goal (DOWN)...')
        future = node.cartesian_client.send_goal_async(cart_goal_msg_down)
        rclpy.spin_until_future_complete(node, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            node.get_logger().error('Cartesian command (DOWN) rejected.')
            return

        # Let it move for 1.5 seconds, then preempt/cancel it
        node.get_logger().info('Moving relative down... Preempting in 1.5 seconds...')
        node.spin_sleep(1.5)
        
        node.get_logger().info('Preempting Cartesian Goal...')
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future)
        node.get_logger().info('Cartesian Goal canceled.')
        
        node.spin_sleep(0.5)

        # Move to Zero Pose via Joint control
        node.get_logger().info('Sending Joint Goal to return to Zero Pose (3.0s)...')
        zero_goal_msg = Rby1JointCommand.Goal()
        zero_goal_msg.right_arm = JointCommand()
        zero_goal_msg.right_arm.position = [0.0] * 7
        zero_goal_msg.right_arm.minimum_time = 3.0
        zero_goal_msg.left_arm = JointCommand()
        zero_goal_msg.left_arm.position = [0.0] * 7
        zero_goal_msg.left_arm.minimum_time = 3.0
        
        future = node.joint_client.send_goal_async(zero_goal_msg)
        rclpy.spin_until_future_complete(node, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            node.get_logger().error('Zero pose joint goal rejected.')
            return
        
        get_res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, get_res_future)
        node.get_logger().info('Returned to Zero Pose successfully.')

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
