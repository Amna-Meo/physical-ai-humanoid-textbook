#!/usr/bin/env python3
"""
Simulation Example: Control Systems for Physical AI
Chapter 7: Control Systems for Physical AI

This example demonstrates various control techniques for Physical AI systems,
including PID control, feedback linearization, and Model Predictive Control.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import control  # python-control package
import math


class PhysicalAIController:
    """
    Demonstrates different control techniques for Physical AI systems
    """

    def __init__(self):
        # System parameters (e.g., for a simple robotic manipulator)
        self.m = 1.0      # mass (kg)
        self.b = 0.4     # damping coefficient
        self.k = 0.2     # spring constant
        self.dt = 0.01   # time step

    def plant_dynamics(self, t, state, u):
        """
        Second-order system dynamics: m*ẍ + b*ẋ + k*x = u
        state = [position, velocity] = [x, ẋ]
        """
        x, v = state
        dxdt = v
        dvdt = (u - self.b*v - self.k*x) / self.m
        return [dxdt, dvdt]

    def simulate_open_loop(self, duration=10.0):
        """
        Simulate open-loop response
        """
        def system_ode(t, state):
            return self.plant_dynamics(t, state, u=1.0)  # Constant input

        initial_state = [0.0, 0.0]  # Initial position and velocity
        t_span = (0, duration)
        t_eval = np.arange(0, duration, self.dt)

        solution = solve_ivp(system_ode, t_span, initial_state,
                           t_eval=t_eval, method='RK45')
        return solution.t, solution.y

    def pid_controller(self, error, error_integral, error_derivative, kp, ki, kd):
        """
        PID controller implementation
        """
        control_output = kp * error + ki * error_integral + kd * error_derivative
        return control_output

    def simulate_pid_control(self, duration=10.0, kp=10.0, ki=1.0, kd=1.0):
        """
        Simulate PID control of the system
        """
        t = np.arange(0, duration, self.dt)
        states = np.zeros((len(t), 2))  # [position, velocity]
        states[0] = [0.0, 0.0]  # Initial state
        control_inputs = np.zeros(len(t))

        # PID state variables
        error_integral = 0
        prev_error = 0
        reference = 1.0  # Desired position

        for i in range(1, len(t)):
            # Calculate error
            current_pos = states[i-1, 0]
            error = reference - current_pos

            # Update integral and derivative
            error_integral += error * self.dt
            error_derivative = (error - prev_error) / self.dt if i > 1 else 0

            # PID control
            u = self.pid_controller(error, error_integral, error_derivative, kp, ki, kd)

            # Apply saturation to control input
            u = np.clip(u, -10, 10)

            # Update system state
            derivatives = self.plant_dynamics(t[i-1], states[i-1], u)
            states[i] = states[i-1] + np.array(derivatives) * self.dt

            control_inputs[i-1] = u
            prev_error = error

        return t, states, control_inputs

    def feedback_linearization_controller(self, state, reference_state, kp=10.0, kd=2.0):
        """
        Feedback linearization controller for second-order system
        """
        x, v = state
        xd, vd = reference_state  # desired position and velocity

        # Desired acceleration using PD control in the linearized space
        desired_accel = kp * (xd - x) + kd * (vd - v)

        # Feedback linearization control law
        # For system: m*ẍ + b*ẋ + k*x = u
        # Control law: u = m*ẍ_d + b*ẋ + k*x - m*ẍ_d
        u = self.m * desired_accel + self.b * v + self.k * x

        return u

    def simulate_feedback_linearization(self, duration=10.0):
        """
        Simulate feedback linearization control
        """
        t = np.arange(0, duration, self.dt)
        states = np.zeros((len(t), 2))  # [position, velocity]
        states[0] = [0.0, 0.0]  # Initial state
        control_inputs = np.zeros(len(t))

        # Trajectory generation - simple step reference
        reference_positions = np.ones(len(t)) * 1.0  # Step to position 1.0
        reference_velocities = np.zeros(len(t))      # Zero velocity reference

        for i in range(1, len(t)):
            # Current reference state
            ref_state = [reference_positions[i], reference_velocities[i]]

            # Feedback linearization control
            u = self.feedback_linearization_controller(states[i-1], ref_state)

            # Apply saturation to control input
            u = np.clip(u, -20, 20)

            # Update system state
            derivatives = self.plant_dynamics(t[i-1], states[i-1], u)
            states[i] = states[i-1] + np.array(derivatives) * self.dt

            control_inputs[i-1] = u

        return t, states, control_inputs

    def simulate_mpc_control(self, duration=10.0, prediction_horizon=10):
        """
        Simulate simplified MPC control (using quadratic programming approach)
        """
        # For simplicity, implementing a basic MPC-like controller
        # that re-computes optimal control based on current state

        t = np.arange(0, duration, self.dt)
        states = np.zeros((len(t), 2))  # [position, velocity]
        states[0] = [0.0, 0.0]  # Initial state
        control_inputs = np.zeros(len(t))

        reference = 1.0  # Desired position

        for i in range(1, len(t)):
            # Simple MPC: compute control based on current error and predicted response
            current_pos, current_vel = states[i-1]
            error = reference - current_pos

            # Simple control law based on current state and desired behavior
            # This is a simplified version of MPC optimization
            kp = 5.0
            kv = 1.0
            u = kp * error - kv * current_vel  # Proportional on position, derivative on velocity

            # Apply saturation
            u = np.clip(u, -15, 15)

            # Update system state
            derivatives = self.plant_dynamics(t[i-1], states[i-1], u)
            states[i] = states[i-1] + np.array(derivatives) * self.dt

            control_inputs[i-1] = u

        return t, states, control_inputs

    def compare_control_methods(self):
        """
        Compare different control methods
        """
        print("Comparing Control Methods for Physical AI Systems")
        print("=" * 50)

        # Simulate different control approaches
        t_pid, states_pid, u_pid = self.simulate_pid_control(duration=10.0)
        t_fl, states_fl, u_fl = self.simulate_feedback_linearization(duration=10.0)
        t_mpc, states_mpc, u_mpc = self.simulate_mpc_control(duration=10.0)

        # Calculate performance metrics
        def calculate_performance(t, states, reference=1.0):
            pos_error = np.abs(states[:, 0] - reference)
            settling_time_idx = np.where(pos_error < 0.02 * reference)[0]  # 2% of reference
            settling_time = t[settling_time_idx[0]] if len(settling_time_idx) > 0 else t[-1]

            rise_time_idx = np.where(states[:, 0] >= 0.1 * reference)[0]
            if len(rise_time_idx) > 0:
                rise_time = t[rise_time_idx[0]]
            else:
                rise_time = t[-1]

            overshoot = max(0, (max(states[:, 0]) - reference) / reference) * 100 if reference != 0 else 0

            return settling_time, rise_time, overshoot

        settling_pid, rise_pid, overshoot_pid = calculate_performance(t_pid, states_pid)
        settling_fl, rise_fl, overshoot_fl = calculate_performance(t_fl, states_fl)
        settling_mpc, rise_mpc, overshoot_mpc = calculate_performance(t_mpc, states_mpc)

        print(f"PID Control:  Settling Time: {settling_pid:.2f}s, Rise Time: {rise_pid:.2f}s, Overshoot: {overshoot_pid:.1f}%")
        print(f"Feedback Linearization: Settling Time: {settling_fl:.2f}s, Rise Time: {rise_fl:.2f}s, Overshoot: {overshoot_fl:.1f}%")
        print(f"MPC Control:  Settling Time: {settling_mpc:.2f}s, Rise Time: {rise_mpc:.2f}s, Overshoot: {overshoot_mpc:.1f}%")

        # Plot comparison
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))

        # Position comparison
        axes[0, 0].plot(t_pid, states_pid[:, 0], label='PID', linewidth=2)
        axes[0, 0].plot(t_fl, states_fl[:, 0], label='Feedback Linearization', linewidth=2)
        axes[0, 0].plot(t_mpc, states_mpc[:, 0], label='MPC', linewidth=2)
        axes[0, 0].axhline(y=1.0, color='k', linestyle='--', label='Reference')
        axes[0, 0].set_title('Position Response Comparison')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Position')
        axes[0, 0].legend()
        axes[0, 0].grid(True)

        # Velocity comparison
        axes[0, 1].plot(t_pid, states_pid[:, 1], label='PID', linewidth=2)
        axes[0, 1].plot(t_fl, states_fl[:, 1], label='Feedback Linearization', linewidth=2)
        axes[0, 1].plot(t_mpc, states_mpc[:, 1], label='MPC', linewidth=2)
        axes[0, 1].set_title('Velocity Response Comparison')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Velocity')
        axes[0, 1].legend()
        axes[0, 1].grid(True)

        # Control effort comparison
        axes[1, 0].plot(t_pid[:-1], u_pid[:-1], label='PID', linewidth=2)
        axes[1, 0].plot(t_fl[:-1], u_fl[:-1], label='Feedback Linearization', linewidth=2)
        axes[1, 0].plot(t_mpc[:-1], u_mpc[:-1], label='MPC', linewidth=2)
        axes[1, 0].set_title('Control Effort Comparison')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Control Input')
        axes[1, 0].legend()
        axes[1, 0].grid(True)

        # Phase portrait
        axes[1, 1].plot(states_pid[:, 0], states_pid[:, 1], label='PID', linewidth=2)
        axes[1, 1].plot(states_fl[:, 0], states_fl[:, 1], label='Feedback Linearization', linewidth=2)
        axes[1, 1].plot(states_mpc[:, 0], states_mpc[:, 1], label='MPC', linewidth=2)
        axes[1, 1].set_title('Phase Portrait (Position vs Velocity)')
        axes[1, 1].set_xlabel('Position')
        axes[1, 1].set_ylabel('Velocity')
        axes[1, 1].legend()
        axes[1, 1].grid(True)

        plt.tight_layout()
        plt.savefig('control_systems_comparison.png', dpi=300, bbox_inches='tight')
        plt.show()

        return {
            'pid': (t_pid, states_pid, u_pid),
            'feedback_linearization': (t_fl, states_fl, u_fl),
            'mpc': (t_mpc, states_mpc, u_mpc)
        }


def main():
    """
    Main function to demonstrate control systems for Physical AI
    """
    print("Control Systems Simulation Example")
    print("Chapter 7: Control Systems for Physical AI")
    print("=" * 50)

    # Create controller
    controller = PhysicalAIController()

    # Compare different control methods
    results = controller.compare_control_methods()

    print("\nControl Systems Analysis Complete")
    print("Visualizations saved to 'control_systems_comparison.png'")


if __name__ == "__main__":
    main()