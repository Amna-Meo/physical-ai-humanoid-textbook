#!/usr/bin/env python3
"""
Simulation Example: Gazebo for Physical AI
Chapter 3: Gazebo Simulation for Physical AI

This example demonstrates Gazebo simulation concepts for Physical AI,
including robot model configuration, sensor simulation, and physics parameters.
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan, Image, JointState
from gazebo_msgs.srv import SetModelState, GetModelState, SpawnModel
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Header
import tf
import math


class GazeboPhysicalAISim:
    """
    Example class demonstrating Gazebo simulation for Physical AI
    """

    def __init__(self):
        rospy.init_node('gazebo_physical_ai_sim', anonymous=True)

        # Publishers for robot control
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.joint_cmd_pub = rospy.Publisher('/joint_commands', JointState, queue_size=10)

        # Subscribers for sensor data
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.camera_sub = rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # Service proxies for Gazebo interaction
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Robot state
        self.robot_position = Vector3(0, 0, 0)
        self.robot_orientation = Vector3(0, 0, 0)
        self.laser_data = None
        self.camera_data = None

        rospy.loginfo("Gazebo Physical AI Simulation initialized")

    def laser_callback(self, data):
        """Process laser scan data"""
        self.laser_data = data
        # Example: Find closest obstacle
        if data.ranges:
            min_distance = min([r for r in data.ranges if not math.isnan(r) and r > 0])
            rospy.loginfo(f"Closest obstacle: {min_distance:.2f}m")

    def camera_callback(self, data):
        """Process camera image data"""
        self.camera_data = data
        # Process image for object detection, etc.

    def imu_callback(self, data):
        """Process IMU data"""
        # Process orientation and acceleration data
        pass

    def move_robot(self, linear_vel, angular_vel):
        """Send velocity commands to robot"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)

    def get_robot_state(self):
        """Get current robot state from Gazebo"""
        try:
            model_state = self.get_model_state('robot', 'world')
            self.robot_position = model_state.pose.position
            self.robot_orientation = model_state.pose.orientation
            return model_state
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def set_robot_state(self, x, y, z, roll, pitch, yaw):
        """Set robot state in Gazebo"""
        try:
            state_msg = ModelState()
            state_msg.model_name = 'robot'
            state_msg.pose.position.x = x
            state_msg.pose.position.y = y
            state_msg.pose.position.z = z

            # Convert Euler to quaternion
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            state_msg.pose.orientation.x = quaternion[0]
            state_msg.pose.orientation.y = quaternion[1]
            state_msg.pose.orientation.z = quaternion[2]
            state_msg.pose.orientation.w = quaternion[3]

            resp = self.set_model_state(state_msg)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def run_simulation(self):
        """Main simulation loop"""
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # Example: Simple obstacle avoidance
            if self.laser_data and min(self.laser_data.ranges) < 1.0:
                # Obstacle detected, turn
                self.move_robot(0.0, 0.5)
            else:
                # Clear path, move forward
                self.move_robot(0.5, 0.0)

            rate.sleep()


def main():
    """Main function to run the Gazebo simulation example"""
    sim = GazeboPhysicalAISim()

    try:
        sim.run_simulation()
    except rospy.ROSInterruptException:
        rospy.loginfo("Simulation terminated")


if __name__ == '__main__':
    main()