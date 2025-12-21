# Chapter 2: ROS 2 for Humanoid Robotics

## Learning Objectives
By the end of this chapter, you should be able to:
- Understand the architecture and core concepts of ROS 2
- Create and manage ROS 2 packages and workspaces
- Implement communication patterns using topics, services, and actions
- Integrate navigation and manipulation capabilities in humanoid robots
- Connect ROS 2 with simulation environments like Gazebo
- Apply safety considerations for humanoid robotics applications

## 2.1 Introduction to ROS 2

The Robot Operating System 2 (ROS 2) represents a significant evolution from its predecessor, designed specifically to address the needs of commercial and safety-critical robotics applications. For humanoid robotics, ROS 2 provides the essential middleware infrastructure that enables complex interactions between perception, planning, control, and execution systems (Macenski et al., 2022).

ROS 2 is not an operating system in the traditional sense but rather a collection of libraries, tools, and conventions that facilitate the development of robot software. The key improvements in ROS 2 over ROS 1 include:

- **Real-time support**: Deterministic behavior for safety-critical applications
- **Multi-robot systems**: Native support for distributed robot systems
- **Security**: Built-in security features for commercial deployment
- **Quality of Service (QoS)**: Configurable communication behavior
- **Cross-platform support**: Improved support for Windows, macOS, and embedded systems

For humanoid robotics, these features are particularly important because humanoid robots typically require integration of numerous sensors and actuators, real-time control capabilities, and robust communication between subsystems. The distributed nature of ROS 2 also makes it well-suited for the complex architectures required in humanoid robots.

### 2.1.1 Evolution from ROS 1 to ROS 2

ROS 1, while revolutionary in its time, had several limitations that made it challenging to use in production environments. These included:

- Single-master architecture that created a single point of failure
- Limited real-time capabilities
- Lack of security features
- Difficulties with multi-robot systems
- Challenges with Quality of Service (QoS) configuration

ROS 2 addresses these limitations by adopting a peer-to-peer architecture based on the Data Distribution Service (DDS) standard. This allows for more robust, scalable, and production-ready robot systems (Macenski et al., 2022).

### 2.1.2 Why ROS 2 for Humanoid Robotics

Humanoid robots present unique challenges that make ROS 2 particularly suitable:

1. **Complex sensor integration**: Humanoid robots typically have numerous sensors (cameras, IMUs, force/torque sensors, joint encoders) that need to communicate efficiently
2. **Distributed control**: Control systems are often distributed across multiple computers
3. **Real-time requirements**: Balance and motion control require deterministic timing
4. **Safety-critical operations**: Humanoid robots operate near humans and require robust safety systems
5. **Modular architecture**: Different subsystems (navigation, manipulation, perception) need to work together

## 2.2 ROS 2 Architecture and Concepts

### 2.2.1 DDS-Based Communication

ROS 2 uses the Data Distribution Service (DDS) as its underlying communication middleware. DDS is an industry-standard specification that provides a publish-subscribe communication model with Quality of Service (QoS) controls. This allows ROS 2 to provide reliable, real-time communication that can be configured for different application needs (Macenski et al., 2022).

The DDS architecture provides several key benefits for humanoid robotics:
- **Reliability**: Configurable reliability policies ensure messages are delivered
- **Real-time performance**: Configurable deadlines and liveliness checks
- **Efficiency**: Zero-copy data sharing between processes
- **Scalability**: Support for large, distributed systems

### 2.2.2 Nodes, Packages, and Workspaces

In ROS 2, a **node** is a process that performs computation. Nodes are organized into **packages**, which contain the source code, configuration files, and other resources needed for a specific functionality. Multiple packages are organized into a **workspace**, which is a directory containing all the packages needed for a robot application.

For humanoid robots, this modular structure is essential because different aspects of the robot (locomotion, manipulation, perception) can be developed as separate packages and integrated at the system level.

### 2.2.3 Client Libraries

ROS 2 provides client libraries that allow nodes to be written in different programming languages. The primary client libraries are:

- **rclcpp**: C++ client library
- **rclpy**: Python client library
- **rclrs**: Rust client library (in development)
- **rclc**: C client library (for embedded systems)

For humanoid robotics, C++ is often preferred for performance-critical components like control loops, while Python is commonly used for higher-level planning and debugging.

### 2.2.4 Quality of Service (QoS) Settings

QoS settings allow fine-tuning of communication behavior in ROS 2. Key QoS settings include:

- **Reliability**: Reliable (all messages delivered) or best-effort (messages may be dropped)
- **Durability**: Whether messages are stored for late-joining subscribers
- **History**: How many messages to store
- **Deadline**: Maximum time between messages
- **Liveliness**: How to detect if a publisher is still active

For humanoid robots, these settings are crucial for ensuring that safety-critical messages (like emergency stops) are reliably delivered while less critical messages (like diagnostic data) can be dropped if necessary.

## 2.3 Core Communication Patterns

### 2.3.1 Topics for Asynchronous Communication

Topics provide a publish-subscribe communication pattern where publishers send messages to a topic and subscribers receive messages from that topic. This is ideal for sensor data streams, where multiple components need to access the same information simultaneously.

For humanoid robots, common topics include:
- `/joint_states`: Current positions, velocities, and efforts of all joints
- `/tf` and `/tf_static`: Transformations between coordinate frames
- `/camera/image_raw`: Raw camera images
- `/imu/data`: Inertial measurement unit data
- `/scan`: Laser scan data

```python
# Example: Publishing joint states in a humanoid robot
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish_joint_states)  # 100 Hz

    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_hip', 'left_knee', 'left_ankle',
                   'right_hip', 'right_knee', 'right_ankle']
        msg.position = [0.1, 0.2, 0.3, 0.1, 0.2, 0.3]
        self.publisher.publish(msg)
```

### 2.3.2 Services for Request-Response Patterns

Services provide synchronous request-response communication, where a client sends a request and waits for a response from a server. This is suitable for operations that have a clear beginning and end, such as setting parameters or requesting a specific action.

For humanoid robots, services might include:
- `/set_joint_position`: Move a joint to a specific position
- `/get_robot_state`: Retrieve the current state of the robot
- `/load_trajectory`: Load a predefined motion trajectory

### 2.3.3 Actions for Long-Running Tasks

Actions are designed for long-running tasks that may take seconds or minutes to complete. They provide feedback during execution and can be preempted if necessary. This is ideal for navigation goals, manipulation tasks, or complex motion sequences.

For humanoid robots, actions commonly include:
- `/move_base_flex`: Navigate to a goal position
- `/move_group`: Plan and execute manipulation motions
- `/joint_trajectory_controller/follow_joint_trajectory`: Execute a sequence of joint positions

```python
# Example: Action client for humanoid walking
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from humanoid_msgs.action import WalkToGoal

class WalkController(Node):
    def __init__(self):
        super().__init__('walk_controller')
        self._action_client = ActionClient(self, WalkToGoal, 'walk_to_goal')

    def send_goal(self, x, y, theta):
        goal_msg = WalkToGoal.Goal()
        goal_msg.target_pose.x = x
        goal_msg.target_pose.y = y
        goal_msg.target_pose.theta = theta

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Walking progress: {feedback.distance_traveled}')
```

## 2.4 ROS 2 Tools and Development Environment

### 2.4.1 Command-Line Tools

ROS 2 provides a comprehensive set of command-line tools that begin with the `ros2` command:

- `ros2 run <package> <executable>`: Run a node
- `ros2 launch <package> <launch_file>`: Launch multiple nodes with configuration
- `ros2 topic`: Monitor and interact with topics
- `ros2 service`: Monitor and interact with services
- `ros2 action`: Monitor and interact with actions
- `ros2 node`: Monitor nodes
- `ros2 pkg`: Manage packages

These tools are essential for debugging and monitoring humanoid robot systems.

### 2.4.2 Visualization with rviz

RViz2 is the 3D visualization tool for ROS 2 that allows developers to visualize robot models, sensor data, and planning results. For humanoid robots, rviz is crucial for:

- Visualizing the robot model in different poses
- Displaying sensor data (laser scans, camera images, point clouds)
- Visualizing planned trajectories
- Debugging coordinate transformations

### 2.4.3 GUI Tools with rqt

rqt provides a framework for creating GUI tools in ROS 2. Common rqt plugins for humanoid robots include:

- rqt_graph: Visualize the node graph
- rqt_plot: Plot numerical data
- rqt_console: View log messages
- rqt_bag: Play back recorded data

## 2.5 Package Management and Workspace Setup

### 2.5.1 Creating ROS 2 Packages

ROS 2 packages are created using the `ros2 pkg create` command:

```bash
ros2 pkg create --build-type ament_cmake --dependencies rclcpp rclpy std_msgs sensor_msgs my_humanoid_robot_package
```

For humanoid robots, typical packages might include:
- `humanoid_description`: Robot URDF and meshes
- `humanoid_control`: Control algorithms and configurations
- `humanoid_navigation`: Navigation stack configuration
- `humanoid_manipulation`: Manipulation capabilities
- `humanoid_bringup`: Launch files for bringing up the robot

### 2.5.2 CMakeLists.txt and package.xml

The `CMakeLists.txt` file defines how the package is built, while `package.xml` contains package metadata and dependencies. For humanoid robots, these files must properly declare dependencies on control libraries, sensor drivers, and other robotics-specific packages.

### 2.5.3 Building and Sourcing Workspaces

ROS 2 workspaces are built using `colcon`:

```bash
colcon build --packages-select my_humanoid_robot_package
source install/setup.bash
```

For humanoid robots, the build process may involve complex dependencies and real-time libraries that require careful configuration.

## 2.6 Navigation in Humanoid Robots

### 2.6.1 Navigation2 Stack Overview

Navigation2 is the official navigation stack for ROS 2, designed to provide robust navigation capabilities for mobile robots. For humanoid robots, Navigation2 provides:

- Path planning algorithms
- Local and global costmaps
- Localization (AMCL)
- Recovery behaviors
- Dynamic obstacle avoidance

The Navigation2 stack is organized into several key components:
- **Planners**: Global and local path planners
- **Controllers**: Trajectory controllers for following paths
- **Recovery**: Behaviors for recovering from failures
- **BT Navigator**: Behavior tree-based navigation logic

### 2.6.2 Costmaps and Path Planning

Costmaps represent the environment as a grid of cells with associated costs for navigation. For humanoid robots, costmaps must account for:

- Robot footprint and dimensions
- Leg placement requirements
- Balance constraints
- Stair and step navigation

```yaml
# Example costmap configuration for humanoid robot
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  resolution: 0.05
  footprint: [[0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3]]
  inflation_radius: 0.55
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}

local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 10.0
  publish_frequency: 5.0
  resolution: 0.025
  footprint: [[0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3]]
  inflation_radius: 0.35
```

### 2.6.3 Localization for Humanoid Robots

Localization determines the robot's position in a known map. For humanoid robots, localization must account for:

- Sensor placement (head-mounted cameras and IMUs)
- Dynamic body movements that can affect sensor readings
- Balance-related movements that change sensor positions

### 2.6.4 Integration with Balance Control

Humanoid navigation must be tightly integrated with balance control systems to ensure stable locomotion. This typically involves:

- Coordinated leg movements for walking
- Center of mass control during navigation
- Step planning that considers balance constraints
- Integration with whole-body controllers

## 2.7 Manipulation and Control

### 2.7.1 MoveIt! Integration with ROS 2

MoveIt! is the standard motion planning framework for ROS 2, providing capabilities for:

- Collision detection and avoidance
- Inverse kinematics
- Trajectory planning
- Grasp planning
- Scene understanding

For humanoid robots, MoveIt! can be used for both arm manipulation and whole-body motion planning that includes balance constraints.

### 2.7.2 Robot State and Motion Planning

The robot state in MoveIt! includes:
- Joint positions and velocities
- End-effector poses
- Collision object representations
- Attached object information

Motion planning for humanoid robots must consider:
- Kinematic constraints
- Dynamic balance requirements
- Collision avoidance
- Task-specific requirements

### 2.7.3 Control Interfaces for Humanoid Robots

ROS 2 provides several control interfaces for humanoid robots:

- **ros2_control**: Standardized control framework
- **Joint trajectory controllers**: Execute sequences of joint positions
- **Position, velocity, and effort controllers**: Different control modes
- **Forward and inverse kinematics**: Coordinate transformations

```yaml
# Example ros2_control configuration for humanoid robot
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_trajectory_controller:
      ros__parameters:
        joints:
          - left_hip_joint
          - left_knee_joint
          - left_ankle_joint
          - right_hip_joint
          - right_knee_joint
          - right_ankle_joint
        interface_name: position
```

### 2.7.4 Trajectory Execution

Trajectory execution involves sending a sequence of joint positions to the robot's controllers. For humanoid robots, this must be done with precise timing to maintain balance and achieve smooth motion.

### 2.7.5 Grasp Planning and Execution

Grasp planning for humanoid robots involves:
- Object recognition and pose estimation
- Grasp pose generation
- Pre-grasp and post-grasp motions
- Force control for secure grasping

## 2.8 Simulation Integration with Gazebo

### 2.8.1 Gazebo-ROS 2 Bridge

The Gazebo-ROS 2 bridge enables communication between Gazebo simulation and ROS 2. This bridge:

- Exposes Gazebo topics as ROS 2 topics
- Converts between Gazebo and ROS message formats
- Provides ROS 2 services for simulation control
- Supports plugins for custom simulation behaviors

### 2.8.2 Robot Description Format (URDF)

URDF (Unified Robot Description Format) describes the robot's physical and kinematic properties:

- Link definitions with mass and inertial properties
- Joint definitions with limits and dynamics
- Visual and collision geometry
- Transmission definitions for control

```xml
<!-- Example URDF snippet for humanoid robot leg -->
<link name="left_hip">
  <inertial>
    <mass value="2.0"/>
    <origin xyz="0 0 0.1"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0"/>
    <geometry>
      <mesh filename="package://humanoid_description/meshes/hip.dae"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0"/>
    <geometry>
      <mesh filename="package://humanoid_description/meshes/hip_collision.dae"/>
    </geometry>
  </collision>
</link>
```

### 2.8.3 Simulation Controllers

Simulation controllers in Gazebo allow for testing control algorithms before deployment on real hardware. These controllers:

- Interface with Gazebo's physics engine
- Provide realistic sensor simulation
- Support various control modes (position, velocity, effort)
- Enable hardware-in-the-loop testing

### 2.8.4 Sensor Integration

Gazebo can simulate various sensors commonly found on humanoid robots:
- Cameras and depth sensors
- IMUs and accelerometers
- Force/torque sensors
- Joint position and velocity sensors
- LIDAR and other range sensors

### 2.8.5 Transfer from Simulation to Reality

The sim-to-real transfer remains one of the key challenges in humanoid robotics. Strategies include:

- Domain randomization during training
- System identification for accurate simulation models
- Robust control algorithms that handle model errors
- Gradual deployment strategies

## 2.9 Real Hardware Integration

### 2.9.1 Hardware Abstraction Layers

ROS 2's `ros2_control` framework provides hardware abstraction through:

- **Hardware interfaces**: Standardized interfaces for different hardware types
- **Resource managers**: Track available hardware resources
- **Controllers**: Standardized control algorithms
- **Controller managers**: Manage controller lifecycle

### 2.9.2 Sensor and Actuator Drivers

Hardware drivers in ROS 2 typically:
- Publish sensor data as ROS 2 topics
- Subscribe to control commands
- Handle hardware-specific communication protocols
- Provide diagnostic information

### 2.9.3 Real-Time Considerations

Real-time performance is critical for humanoid robot control:

- **Real-time kernel**: Use PREEMPT_RT patched Linux kernels
- **Memory management**: Lock memory to prevent page faults
- **Priority scheduling**: Use SCHED_FIFO for critical control loops
- **Deterministic communication**: Configure DDS QoS for real-time performance

### 2.9.4 Safety Systems

Safety systems for humanoid robots typically include:
- Emergency stop mechanisms
- Collision detection and avoidance
- Joint limit enforcement
- Balance recovery behaviors
- Human safety protocols

## 2.10 Safety and Best Practices

### 2.10.1 Safety Considerations for Humanoid Robots

Safety in humanoid robotics requires multiple layers of protection:

- **Physical safety**: Preventing harm to humans and the robot
- **Operational safety**: Safe operation in various environments
- **Software safety**: Robust error handling and recovery
- **Communication safety**: Secure and reliable communication

### 2.10.2 Error Handling and Fault Tolerance

ROS 2 systems should implement:
- Graceful degradation when components fail
- Health monitoring and diagnostics
- Automatic recovery from common failures
- Clear error reporting and logging

### 2.10.3 Testing Strategies

Comprehensive testing for humanoid robots includes:
- Unit testing for individual components
- Integration testing for subsystems
- Simulation testing for safety-critical behaviors
- Hardware testing for real-world validation

### 2.10.4 Performance Optimization

Performance optimization strategies:
- Efficient message passing with appropriate QoS settings
- Proper node organization to minimize communication overhead
- Use of zero-copy data sharing where possible
- Profiling and monitoring tools for performance analysis

## 2.11 Summary

ROS 2 provides the essential middleware infrastructure for developing humanoid robot systems. Its peer-to-peer architecture based on DDS, real-time capabilities, and security features make it well-suited for the complex requirements of humanoid robotics. The modular package structure, comprehensive toolset, and integration with simulation environments enable efficient development and deployment of humanoid robot applications.

The Navigation2 stack provides robust navigation capabilities, while MoveIt! enables sophisticated manipulation and motion planning. The integration with Gazebo allows for safe development and testing before deployment on real hardware. The `ros2_control` framework provides standardized interfaces for hardware integration.

As we continue through this textbook, we will explore how these ROS 2 capabilities are applied in specific humanoid robot applications, from basic locomotion to complex manipulation tasks. The next chapter will focus on Gazebo simulation, building on the ROS 2 foundation we have established here.

## References

Bou-Ammar, H., Busoniu, L., Dornhege, C., & Neumann, G. (2014). Reinforcement learning for humanoid robotics. *IEEE-RAS International Conference on Humanoid Robots*.

Coleman, G., Suárez, J. P., Tapus, A., & Billard, A. (2014). Improving human-robot interaction in ROS using a 3D camera. *International Conference on Simulation, Modeling, and Programming for Autonomous Robots*.

Coltin, B., & Veloso, M. (2014). Interactive object acquisition for manipulation. *IEEE Robotics and Automation Letters*, 1(1), 28-35.

Dornhege, C., Hertle, F., & Ferrein, A. (2013). Navigation for ROS: The navigation stack. *RoboCup 2012: Robot Soccer World Cup XVI*, 24-35.

Ferrein, A., & Lakemeyer, G. (2008). Deliberation for navigation in ROS. *Workshop on Semantic Robot Perception*.

Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IEEE/RSJ International Conference on Intelligent Robots and Systems*.

Koubaa, A. (2018). *ROS robotics projects*. Packt Publishing.

Lynch, K. M., & Park, F. C. (2017). *Modern robotics*. Cambridge University Press.

Macenski, S., Woodall, S., & Faust, J. (2022). ROS 2: Next Generation Robot Middleware. *IEEE International Conference on Robotics and Automation (ICRA)*.

Mason, Z., Diankov, R., Padır, T., & Amato, N. (2012). Open humanoids: A common platform for programmatic physical interaction. *IEEE International Conference on Robotics and Automation*.

Nakanishi, J., Cory, R., Mistry, M., Peters, J., & Schaal, S. (2008). Operational space control: A theoretical and empirical comparison. *The International Journal of Robotics Research*, 27(6), 737-757.

Open Robotics. (2023). *ROS 2 Tutorials*. https://docs.ros.org/en/rolling/Tutorials.html

Peters, S., & Cook, J. (2020). *Effective Robotics Programming with ROS*. Packt Publishing.

Quigley, M., Conley, K., Gerkey, B., Faust, J., Foote, T., Leibs, J., ... & Ng, A. Y. (2009). ROS: an open-source Robot Operating System. *ICRA Workshop on Open Source Software*, 3(3.2), 5.

Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming robots with ROS: A practical introduction to the Robot Operating System*. O'Reilly Media.

ROS 2 Design Documents. (2023). Architecture and design principles. https://design.ros2.org/

ROS 2 Documentation. (2023). Robot Operating System 2. https://docs.ros.org/en/rolling/

ROS Safety Working Group. (2023). Safety guidelines for ROS-based systems.

ROS-Industrial Consortium. (2023). ROS-Industrial best practices and guidelines.

Santos, V., Matos, N., & Lau, N. (2013). A behavior-based architecture for robot autonomy using ROS. *ROBOCOMP*.

Stoneman, S., & Axinte, D. (2014). A framework for manipulation in ROS. *IEEE International Conference on Robotics and Biomimetics*.

Wheeler, K., & Chen, I. C. (2020). Real-time control of legged robots using ROS 2. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*.