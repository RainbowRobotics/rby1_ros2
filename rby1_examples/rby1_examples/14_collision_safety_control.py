#!/usr/bin/env python3
"""
Collision Safety Control Example
================================
Demonstrates enabling and disabling the driver's collision safety retreat feature
using the `/set_collision_safety` service, then commands a self-colliding arm pose
to verify the automatic safe return back to the initial zero pose.

Run:
  ros2 run rby1_examples collision_safety_control
"""
import sys
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rby1_msgs.action import Rby1JointCommand
from rby1_msgs.msg import JointCommand, RobotState
from rby1_msgs.srv import StateOnOff

class CollisionSafetyExample(Node):
    def __init__(self):
        super().__init__('collision_safety_example')
        self._action_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.safety_client = self.create_client(StateOnOff, 'set_collision_safety')
        
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
        self.get_logger().info('Checking robot power and servo status...')
        rclpy.spin_once(self, timeout_sec=0.5)
        if self.control_state in [2, 3]:
            self.get_logger().info('Robot is already enabled.')
            return True

        # Power On
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
        
        time.sleep(2.0)
            
        # Servo On
        self.get_logger().info('Sending Servo ON request...')
        self.servo_client.wait_for_service()
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to servo on: {future.result().message}')
            return False
            
        if self.wait_for_state([2, 3], timeout=15.0):
            self.get_logger().info('Robot is ready.')
            time.sleep(1.0)
            return True
        else:
            self.get_logger().error(f'Timed out waiting for robot to enable. Current state: {self.control_state}')
            return False

    def send_safety_request(self, state):
        req = StateOnOff.Request()
        req.state = state
        req.parameters = ""
        self.safety_client.wait_for_service()
        future = self.safety_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_joint_goal(self, torso_pos, right_pos, left_pos, head_pos, minimum_time):
        goal_msg = Rby1JointCommand.Goal()
        
        if torso_pos is not None:
            goal_msg.torso = JointCommand()
            goal_msg.torso.position = torso_pos
            goal_msg.torso.minimum_time = minimum_time
            
        if right_pos is not None:
            goal_msg.right_arm = JointCommand()
            goal_msg.right_arm.position = right_pos
            goal_msg.right_arm.minimum_time = minimum_time
            
        if left_pos is not None:
            goal_msg.left_arm = JointCommand()
            goal_msg.left_arm.position = left_pos
            goal_msg.left_arm.minimum_time = minimum_time
            
        if head_pos is not None:
            goal_msg.head = JointCommand()
            goal_msg.head.position = head_pos
            goal_msg.head.minimum_time = minimum_time

        goal_msg.priority = 10
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CollisionSafetyExample()

    # 1. Ensure Robot is Ready
    if not node.ensure_robot_ready():
        node.get_logger().error('Robot initialization failed. Exiting.')
        return

    # 2. Enable collision safety option via service
    node.get_logger().info('Enabling collision safety retreat service...')
    res = node.send_safety_request(True)
    if res.success:
        node.get_logger().info(f'Collision safety enabled successfully: {res.message}')
    else:
        node.get_logger().error(f'Failed to enable collision safety: {res.message}')
        return

    # 3. Move to initial safe Zero Pose
    node.get_logger().info('Step 1: Moving to Zero Pose (initial safe pose)...')
    torso_pos = [0.0] * 6
    right_pos = [0.0] * 7
    left_pos = [0.0] * 7
    head_pos = [0.0] * 2
    min_time = 4.0
    
    future = node.send_joint_goal(torso_pos, right_pos, left_pos, head_pos, min_time)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error('Zero pose goal was rejected.')
        return
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)
    if not get_result_future.result().result.success:
        node.get_logger().error('Failed to reach Zero Pose.')
        return
    node.get_logger().info('Successfully reached Zero Pose. Zero pose saved as initial pose.')
    time.sleep(1.0)

    # 4. Command colliding pose to trigger safety retreat
    node.get_logger().info('Step 2: Sending a self-colliding command (arms crossing shoulder/yaw joints)...')
    
    # Command right arm to cross to the left, and left arm to cross to the right
    # Joints 1, 2, 3 of right arm: [0.5, 0.8, -0.5, -1.57, 0.0, 0.0, 0.0]
    # Joints 1, 2, 3 of left arm: [0.5, -0.8, 0.5, -1.57, 0.0, 0.0, 0.0]
    colliding_right_pos = [0.0, 0.8, -0.5, -1.57, 0.0, 0.0, 0.0]
    colliding_left_pos = [0.0, -0.8, 0.5, -1.57, 0.0, 0.0, 0.0]
    
    future = node.send_joint_goal(torso_pos, colliding_right_pos, colliding_left_pos, head_pos, 8.0)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().warn('Colliding pose goal rejected (this might happen if collision is detected immediately at startup).')
        return
    
    node.get_logger().info('Colliding goal sent. Waiting for collision detection to trigger automatic retreat...')
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)
    
    result = get_result_future.result().result
    node.get_logger().info(f'Action result received. Success: {result.success}, Finish code: {result.finish_code}')
    node.get_logger().info('If the safety retreat triggered, the robot should have stopped the colliding motion and returned to Zero Pose.')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
