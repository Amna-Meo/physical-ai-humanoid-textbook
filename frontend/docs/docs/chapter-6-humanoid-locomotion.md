# Chapter 6: Humanoid Locomotion

## Learning Objectives
By the end of this chapter, you should be able to:
- Understand the biomechanics and control principles underlying humanoid locomotion
- Implement basic walking gaits using various control strategies
- Analyze stability and balance in dynamic walking scenarios
- Design controllers for different terrains and walking patterns
- Integrate sensory feedback for robust locomotion
- Evaluate the performance of locomotion algorithms

## 6.1 Introduction to Humanoid Locomotion

Humanoid locomotion represents one of the most challenging problems in robotics, requiring the integration of complex control theory, biomechanics, and real-time computation. Unlike wheeled or tracked robots, humanoid robots must achieve stable locomotion using a limited number of contact points with the ground, while maintaining balance and adapting to environmental conditions (Kajita et al., 2014).

The fundamental challenge in humanoid locomotion lies in the underactuated nature of the system during walking phases. When only one foot is in contact with the ground, the robot has fewer actuators than degrees of freedom, making it inherently unstable. This requires sophisticated control strategies that can manage the dynamic balance while achieving forward motion.

### 6.1.1 Key Challenges in Humanoid Locomotion

The primary challenges in humanoid locomotion include:

- **Balance maintenance**: Keeping the robot's center of mass within the support polygon during dynamic motion
- **Terrain adaptation**: Adjusting gait patterns for different surfaces, slopes, and obstacles
- **Real-time control**: Computing control actions within strict timing constraints to prevent falls
- **Energy efficiency**: Minimizing power consumption while maintaining stable locomotion
- **Robustness**: Handling disturbances and maintaining stability in the presence of external forces

## 6.2 Biomechanics of Human Walking

### 6.2.1 The Human Gait Cycle

Human walking follows a well-defined gait cycle that can be used as inspiration for robotic systems. The gait cycle consists of two main phases:

1. **Stance phase** (60% of cycle): When the foot is in contact with the ground
2. **Swing phase** (40% of cycle): When the foot is off the ground and moving forward

The gait cycle can be further subdivided into:
- Initial contact
- Loading response
- Mid-stance
- Terminal stance
- Pre-swing
- Initial swing
- Mid-swing
- Terminal swing

### 6.2.2 Center of Mass and Zero Moment Point (ZMP)

The Zero Moment Point (ZMP) is a crucial concept in humanoid locomotion, representing the point on the ground where the sum of all moments due to external forces equals zero. For stable walking, the ZMP must remain within the support polygon defined by the feet contact points.

The relationship between the center of mass (CoM) position and ZMP is given by:
ZMP_x = x - (z_h * ẍ) / g
ZMP_y = y - (z_h * ÿ) / g

Where (x, y, z) is the CoM position, z_h is the CoM height, g is gravitational acceleration, and (ẍ, ÿ) are the CoM accelerations.

## 6.3 Walking Pattern Generation

### 6.3.1 Inverted Pendulum Models

The Linear Inverted Pendulum Model (LIPM) is a widely used approach for generating walking patterns. It assumes the robot's mass is concentrated at a single point at a constant height, simplifying the dynamics to:

ẍ = g * (x - zmp) / z_h

This model allows for the generation of stable walking patterns by planning ZMP trajectories that remain within the support polygon.

### 6.3.2 Footstep Planning

Footstep planning involves determining the sequence of foot positions and timing for stable locomotion. Key considerations include:

- Step length and width for stability
- Swing trajectory planning to avoid obstacles
- Timing synchronization between steps
- Adaptation to terrain characteristics

## 6.4 Control Strategies for Locomotion

### 6.4.1 Model Predictive Control (MPC)

Model Predictive Control has emerged as a powerful approach for humanoid locomotion, allowing for the optimization of future states based on a dynamic model. MPC controllers can handle constraints on ZMP, joint angles, and velocities while optimizing for stability and energy efficiency.

The MPC problem for walking can be formulated as:
minimize Σ(x_k - x_ref_k)ᵀQ(x_k - x_ref_k) + u_kᵀRu_k
subject to system dynamics and constraints

### 6.4.2 Capture Point Control

The Capture Point (CP) is a point on the ground where the robot can come to a complete stop with one step. It extends the ZMP concept by considering the robot's current state and the dynamics of stopping.

CP_x = x + v * √(z_h / g)
CP_y = y + u * √(z_h / g)

Where (v, u) are the CoM velocities in x and y directions.

### 6.4.3 Feedback Control for Disturbance Rejection

Robust locomotion requires feedback control mechanisms to handle disturbances and modeling uncertainties. Common approaches include:

- State feedback control for CoM regulation
- Foot placement strategies for balance recovery
- Impedance control for compliant interaction
- Adaptive control for parameter uncertainties

## 6.5 Advanced Locomotion Techniques

### 6.5.1 Dynamic Walking

Dynamic walking involves walking patterns where the robot is always in motion and relies on dynamic balance rather than static stability. This approach can achieve higher speeds and more human-like gait patterns but requires more sophisticated control.

### 6.5.2 Multi-Contact Locomotion

Multi-contact locomotion uses additional contact points beyond the feet (such as hands on walls or railings) to enhance stability and enable more complex movements. This is particularly useful for climbing or navigating challenging terrain.

### 6.5.3 Learning-Based Approaches

Recent advances in machine learning have enabled the development of learning-based locomotion controllers that can adapt to different terrains and learn from experience. These approaches include:

- Reinforcement learning for gait optimization
- Imitation learning from human motion capture
- Neural network controllers for complex behaviors
- Evolutionary algorithms for gait discovery

## 6.6 Implementation Considerations

### 6.6.1 Hardware Constraints

The locomotion controller must account for hardware limitations including:
- Joint torque and velocity limits
- Actuator bandwidth and response time
- Sensor accuracy and update rates
- Computational resources and real-time constraints

### 6.6.2 Safety Considerations

Safety is paramount in humanoid locomotion, requiring:
- Fall detection and mitigation strategies
- Emergency stop procedures
- Collision avoidance mechanisms
- Safe recovery from disturbances

## 6.7 Simulation and Testing

### 6.7.1 Simulation Environments

Simulation is essential for developing and testing locomotion controllers before deployment on hardware. Key simulation tools include:
- Gazebo with physics engines (ODE, Bullet, DART)
- MuJoCo for high-fidelity simulation
- Webots for comprehensive robot simulation
- Custom simulators for specific applications

### 6.7.2 Transfer to Hardware

The sim-to-real transfer remains a significant challenge in humanoid locomotion. Techniques to improve transfer include:
- Domain randomization
- System identification and model refinement
- Robust control design
- Careful parameter tuning

## 6.8 Chapter Summary

This chapter has covered the fundamental concepts of humanoid locomotion, from biomechanics to advanced control strategies. We've explored the challenges of dynamic balance, the importance of ZMP and capture point concepts, and various control approaches including MPC and learning-based methods. The implementation of robust locomotion requires careful consideration of hardware constraints, safety requirements, and the challenges of transferring from simulation to real hardware.

## Exercises

1. Implement a simple ZMP-based walking pattern generator for a 2D walking motion.
2. Compare the stability characteristics of different walking gaits using simulation.
3. Design a feedback controller to reject disturbances during walking.
4. Investigate the effects of step length and timing on walking stability.
5. Implement a capture point-based balance controller for disturbance recovery.

## Further Reading

- Kajita, S., Kanehiro, F., Kaneko, K., Fujiwara, K., & Harada, K. (2014). Introduction to Humanoid Robotics.
- Pratt, J., & Tedrake, R. (2006). On limit cycles and the "safe stepping" problem in the control of bipedal walking robots.
- Englsberger, J., Ott, C., & Schaub, A. (2015). Three-dimensional bipedal walking control using Divergent Component of Motion.
- Wieber, P. B. (2006). Pattern generators with sensory feedback for the control of quadruped and biped walking.