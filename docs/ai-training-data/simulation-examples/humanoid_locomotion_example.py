#!/usr/bin/env python3
"""
Simulation Example: Humanoid Locomotion
Chapter 6: Humanoid Locomotion

This example demonstrates basic humanoid locomotion concepts,
including ZMP-based walking, inverse kinematics, and balance control.
"""

import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import integrate
import time


class HumanoidLocomotionController:
    """
    Controller for humanoid locomotion based on ZMP (Zero Moment Point) theory
    """

    def __init__(self, robot_height=0.8, gravity=9.81):
        self.robot_height = robot_height  # Center of mass height
        self.gravity = gravity
        self.omega = math.sqrt(gravity / robot_height)  # Natural frequency

        # Walking parameters
        self.step_length = 0.3  # meters
        self.step_width = 0.2   # meters
        self.step_height = 0.1  # meters
        self.walk_period = 1.0  # seconds per step

        # Support polygon (foot positions)
        self.left_foot = np.array([0, self.step_width/2, 0])
        self.right_foot = np.array([0, -self.step_width/2, 0])

        # Trajectory storage
        self.com_trajectory = []
        self.zmp_trajectory = []
        self.foot_trajectory = []

    def compute_zmp_reference(self, t, walk_speed=0.2):
        """
        Compute reference ZMP trajectory for walking
        """
        # Simple walking pattern: alternating support polygons
        step_phase = (t % (2 * self.walk_period)) / (2 * self.walk_period)
        step_number = int(t / self.walk_period)

        # Determine support foot
        if step_number % 2 == 0:
            # Left foot support
            support_center = np.array([step_number * self.step_length / 2, self.step_width/2, 0])
        else:
            # Right foot support
            support_center = np.array([step_number * self.step_length / 2, -self.step_width/2, 0])

        # Add some offset for stability
        zmp_x = support_center[0] + walk_speed * self.walk_period / 4  # Move forward in support area
        zmp_y = support_center[1]

        return np.array([zmp_x, zmp_y, 0])

    def plan_com_trajectory(self, t):
        """
        Plan Center of Mass trajectory based on ZMP reference
        Using Linear Inverted Pendulum Model (LIPM)
        """
        # Get reference ZMP
        zmp_ref = self.compute_zmp_reference(t)

        # For LIPM: CoM height is constant
        z_com = self.robot_height

        # Simple CoM planning: follow ZMP with some offset
        com_x = zmp_ref[0]  # CoM x follows ZMP x
        com_y = zmp_ref[1]  # CoM y follows ZMP y
        com_z = z_com       # Constant height

        return np.array([com_x, com_y, com_z])

    def compute_foot_trajectory(self, t, foot_type="left"):
        """
        Compute foot trajectory for stepping motion
        """
        step_number = int(t / self.walk_period)
        step_phase = (t % self.walk_period) / self.walk_period

        if foot_type == "left":
            foot_offset = self.step_width / 2
        else:
            foot_offset = -self.step_width / 2

        # Calculate nominal foot position
        nominal_x = step_number * self.step_length
        nominal_y = foot_offset
        nominal_z = 0  # On ground

        # If this is a swing phase for this foot
        if (step_number % 2 == 0 and foot_type == "left") or \
           (step_number % 2 == 1 and foot_type == "right"):
            # This foot is swinging
            if step_phase < 0.5:
                # Moving to next position
                x = nominal_x + self.step_length * step_phase * 2
                z = self.step_height * math.sin(math.pi * step_phase * 2)  # Parabolic trajectory
            else:
                # Landing
                x = nominal_x + self.step_length
                z = self.step_height * math.sin(math.pi * (1 - step_phase) * 2)  # Parabolic trajectory
            y = nominal_y
        else:
            # This foot is in support, stays in place
            x = nominal_x - self.step_length if step_number > 0 else 0
            y = nominal_y
            z = 0

        return np.array([x, y, z])

    def compute_inverse_kinematics(self, foot_pos, leg_type="left"):
        """
        Simple 3DOF leg inverse kinematics
        """
        # Simplified 3DOF model: hip abduction/adduction, hip flexion/extension, knee
        x, y, z = foot_pos

        # Assume simple planar leg for demonstration
        # Hip x, y relative to body
        hip_x = 0  # Body is at origin
        hip_y = self.step_width/2 if leg_type == "left" else -self.step_width/2
        hip_z = self.robot_height

        # Compute leg length
        dx = x - hip_x
        dy = y - hip_y
        dz = z - hip_z

        # Distance from hip to foot in x-z plane
        r = math.sqrt(dx*dx + dz*dz)

        # Hip flexion angle
        hip_flexion = math.atan2(dx, -dz)

        # Knee angle (assuming simple 2-link leg)
        leg_length = math.sqrt(r*r + dy*dy)
        # Assume fixed thigh and shank lengths for demo
        thigh_length = 0.4
        shank_length = 0.4

        # Law of cosines for knee angle
        cos_knee = (thigh_length**2 + shank_length**2 - leg_length**2) / (2 * thigh_length * shank_length)
        knee_angle = math.pi - math.acos(max(-1, min(1, cos_knee)))

        # Hip abduction for y movement
        hip_abduction = math.atan2(dy, r)

        return np.array([hip_abduction, hip_flexion, knee_angle])

    def generate_walking_pattern(self, duration=10.0, dt=0.01):
        """
        Generate complete walking pattern
        """
        time_points = np.arange(0, duration, dt)
        com_positions = []
        zmp_positions = []
        left_foot_positions = []
        right_foot_positions = []
        left_joint_angles = []
        right_joint_angles = []

        for t in time_points:
            # Compute CoM and ZMP
            com_pos = self.plan_com_trajectory(t)
            zmp_pos = self.compute_zmp_reference(t)

            # Compute foot positions
            left_foot_pos = self.compute_foot_trajectory(t, "left")
            right_foot_pos = self.compute_foot_trajectory(t, "right")

            # Compute joint angles using inverse kinematics
            left_joints = self.compute_inverse_kinematics(left_foot_pos, "left")
            right_joints = self.compute_inverse_kinematics(right_foot_pos, "right")

            # Store trajectories
            com_positions.append(com_pos)
            zmp_positions.append(zmp_pos)
            left_foot_positions.append(left_foot_pos)
            right_foot_positions.append(right_foot_pos)
            left_joint_angles.append(left_joints)
            right_joint_angles.append(right_joints)

        return {
            'time': time_points,
            'com': np.array(com_positions),
            'zmp': np.array(zmp_positions),
            'left_foot': np.array(left_foot_positions),
            'right_foot': np.array(right_foot_positions),
            'left_joints': np.array(left_joint_angles),
            'right_joints': np.array(right_joints)
        }

    def plot_trajectories(self, trajectories):
        """
        Plot the generated walking trajectories
        """
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))

        # CoM trajectory
        axes[0, 0].plot(trajectories['com'][:, 0], trajectories['com'][:, 1])
        axes[0, 0].set_title('CoM Trajectory (Top View)')
        axes[0, 0].set_xlabel('X (m)')
        axes[0, 0].set_ylabel('Y (m)')
        axes[0, 0].grid(True)

        # ZMP vs CoM X
        axes[0, 1].plot(trajectories['time'], trajectories['com'][:, 0], label='CoM X')
        axes[0, 1].plot(trajectories['time'], trajectories['zmp'][:, 0], label='ZMP X')
        axes[0, 1].set_title('X Position vs Time')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Position (m)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)

        # ZMP vs CoM Y
        axes[0, 2].plot(trajectories['time'], trajectories['com'][:, 1], label='CoM Y')
        axes[0, 2].plot(trajectories['time'], trajectories['zmp'][:, 1], label='ZMP Y')
        axes[0, 2].set_title('Y Position vs Time')
        axes[0, 2].set_xlabel('Time (s)')
        axes[0, 2].set_ylabel('Position (m)')
        axes[0, 2].legend()
        axes[0, 2].grid(True)

        # Foot trajectories
        axes[1, 0].plot(trajectories['left_foot'][:, 0], trajectories['left_foot'][:, 1], label='Left Foot')
        axes[1, 0].plot(trajectories['right_foot'][:, 0], trajectories['right_foot'][:, 1], label='Right Foot')
        axes[1, 0].set_title('Foot Trajectories (Top View)')
        axes[1, 0].set_xlabel('X (m)')
        axes[1, 0].set_ylabel('Y (m)')
        axes[1, 0].legend()
        axes[1, 0].grid(True)

        # Joint angles
        axes[1, 1].plot(trajectories['time'], trajectories['left_joints'][:, 0], label='Left Hip Abduction')
        axes[1, 1].plot(trajectories['time'], trajectories['left_joints'][:, 1], label='Left Hip Flexion')
        axes[1, 1].plot(trajectories['time'], trajectories['left_joints'][:, 2], label='Left Knee')
        axes[1, 1].set_title('Left Leg Joint Angles')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Angle (rad)')
        axes[1, 1].legend()
        axes[1, 1].grid(True)

        # Support polygon visualization
        axes[1, 2].plot(trajectories['zmp'][:, 0], trajectories['zmp'][:, 1], 'r-', label='ZMP Path', alpha=0.7)
        axes[1, 2].scatter(trajectories['com'][:, 0], trajectories['com'][:, 1], c='blue', s=1, label='CoM Path', alpha=0.5)
        axes[1, 2].set_title('ZMP vs CoM (Stability)')
        axes[1, 2].set_xlabel('X (m)')
        axes[1, 2].set_ylabel('Y (m)')
        axes[1, 2].legend()
        axes[1, 2].grid(True)
        axes[1, 2].axis('equal')

        plt.tight_layout()
        plt.savefig('humanoid_locomotion_trajectories.png', dpi=300, bbox_inches='tight')
        plt.show()


def main():
    """
    Main function to demonstrate humanoid locomotion
    """
    print("Humanoid Locomotion Simulation Example")
    print("Chapter 6: Humanoid Locomotion")
    print("=" * 50)

    # Create locomotion controller
    controller = HumanoidLocomotionController()

    print("Generating walking pattern for 8 seconds...")
    trajectories = controller.generate_walking_pattern(duration=8.0, dt=0.02)

    print(f"Generated {len(trajectories['time'])} time steps")
    print("CoM trajectory range: X [{:.2f}, {:.2f}] Y [{:.2f}, {:.2f}]".format(
        trajectories['com'][:, 0].min(), trajectories['com'][:, 0].max(),
        trajectories['com'][:, 1].min(), trajectories['com'][:, 1].max()
    ))
    print("Walking distance: {:.2f} meters".format(trajectories['com'][-1, 0]))

    # Plot the results
    controller.plot_trajectories(trajectories)

    print("Simulation complete. Trajectories saved to 'humanoid_locomotion_trajectories.png'")


if __name__ == "__main__":
    main()