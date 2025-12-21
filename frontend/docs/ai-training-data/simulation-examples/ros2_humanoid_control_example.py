#!/usr/bin/env python3
"""
Simulation Example: ROS 2 Control for Humanoid Robotics
Chapter 2: ROS 2 for Humanoid Robotics

This example demonstrates basic ROS 2 concepts applied to humanoid robot control,
including topics, services, actions, and integration with simulation environments.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time


class HumanoidController(Node):
    """
    A simple humanoid robot controller demonstrating ROS 2 concepts
    """

    def __init__(self):
        super().__init__('humanoid_controller')

        # Publishers for different control interfaces
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscribers for sensor feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.controller_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_controller/state',
            self.controller_state_callback,
            10
        )

        # Timer for periodic control updates
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot state tracking
        self.current_joint_positions = {}
        self.target_joint_positions = {}

        self.get_logger().info('Humanoid Controller initialized')

    def joint_state_callback(self, msg):
        """Callback for joint state updates"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def controller_state_callback(self, msg):
        """Callback for controller state updates"""
        # Process controller feedback
        pass

    def send_joint_trajectory(self, joint_names, positions, durations):
        """Send joint trajectory command"""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(durations)
        point.time_from_start.nanosec = int((durations % 1) * 1e9)

        traj_msg.points = [point]

        self.joint_cmd_pub.publish(traj_msg)

    def control_loop(self):
        """Main control loop"""
        # Example: Simple walking gait pattern
        self.execute_simple_walk()


def main(args=None):
    """Main function to run the humanoid controller"""
    rclpy.init(args=args)

    controller = HumanoidController()

    # Example: Send a simple joint command
    joint_names = ['left_hip_joint', 'right_hip_joint', 'left_knee_joint', 'right_knee_joint']
    positions = [0.1, 0.1, 0.0, 0.0]  # Simple starting position
    controller.send_joint_trajectory(joint_names, positions, 2.0)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down humanoid controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()