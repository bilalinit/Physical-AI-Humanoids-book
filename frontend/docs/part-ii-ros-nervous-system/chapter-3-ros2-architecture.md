---
sidebar_position: 2
title: 'Chapter 3: ROS 2 Architecture'
---

# Chapter 3: ROS 2 Architecture

## Learning Objectives

By the end of this chapter, you should be able to:
- Understand the core concepts of ROS 2 architecture
- Create and implement ROS 2 nodes for humanoid robot control
- Design message passing and service architectures
- Implement action servers and clients for long-running tasks
- Understand Quality of Service (QoS) settings for real-time robotics

## Prerequisites

Before starting this chapter, you should:
- Complete Chapter 1 and 2 (Hardware & OS, Jetson Setup)
- Basic understanding of Python or C++ programming
- Familiarity with distributed systems concepts

## Introduction

Robot Operating System 2 (ROS 2) serves as the nervous system for humanoid robots, providing the communication infrastructure that connects sensors, actuators, and intelligence modules. This chapter explores the architecture of ROS 2 and how to implement it for humanoid robotics applications using NVIDIA's ecosystem.

## ROS 2 vs ROS 1: Key Differences

ROS 2 addresses several limitations of ROS 1, particularly important for humanoid robotics:

- **Real-time support**: Better real-time capabilities for critical robot control
- **Multi-robot systems**: Improved support for multi-robot coordination
- **Security**: Built-in security features for safe robot operation
- **Quality of Service (QoS)**: Configurable reliability and performance options
- **DDS-based communication**: Modern middleware for robust communication

## Core Architecture Concepts

### Nodes

Nodes are the fundamental computational units in ROS 2. Each node typically performs a specific function in the robot's operation:

```python
# Example ROS 2 node for humanoid robot joint control
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Publishers for joint commands
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscribers for joint feedback
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.01, self.control_loop)  # 100Hz

        self.get_logger().info('Joint Controller node initialized')

    def joint_state_callback(self, msg):
        # Process joint state feedback
        self.get_logger().info(f'Received {len(msg.name)} joints')

    def control_loop(self):
        # Implement control algorithm here
        pass

def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics and Message Passing

Topics enable asynchronous communication between nodes. For humanoid robotics, this is crucial for sensor data distribution and control command propagation:

```python
# Publisher example: IMU sensor data
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(0.02, self.publish_imu_data)  # 50Hz

    def publish_imu_data(self):
        msg = Imu()
        # Populate message with actual sensor data
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        # Add orientation, angular velocity, linear acceleration data
        self.publisher.publish(msg)
```

### Services

Services provide synchronous request-response communication, useful for configuration and non-real-time operations:

```python
# Service example: Robot calibration
from rclpy.node import Node
from example_interfaces.srv import Trigger

class CalibrationService(Node):
    def __init__(self):
        super().__init__('calibration_service')
        self.srv = self.create_service(
            Trigger,
            'calibrate_robot',
            self.calibrate_callback
        )

    def calibrate_callback(self, request, response):
        self.get_logger().info('Starting calibration procedure...')
        # Perform calibration steps
        success = self.perform_calibration()

        response.success = success
        response.message = 'Calibration completed' if success else 'Calibration failed'
        return response
```

### Actions

Actions are perfect for long-running tasks with feedback, common in humanoid robotics:

```python
# Action example: Walking pattern execution
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory

class WalkingActionServer(Node):
    def __init__(self):
        super().__init__('walking_action_server')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'execute_walking_pattern',
            self.execute_walking_callback
        )

    def execute_walking_callback(self, goal_handle):
        self.get_logger().info('Executing walking pattern...')

        # Simulate walking execution with feedback
        feedback_msg = FollowJointTrajectory.Feedback()

        for i in range(0, 100):
            if goal_handle.is_canceling():
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            # Send feedback
            feedback_msg.actual.time_from_start.sec = i
            goal_handle.publish_feedback(feedback_msg)

            # Simulate walking step
            time.sleep(0.1)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result
```

## Quality of Service (QoS) in Robotics

QoS settings are crucial for humanoid robotics to ensure proper timing and reliability:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For critical control commands (e.g., emergency stop)
critical_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# For sensor data (e.g., camera feed)
sensor_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# Create subscriber with specific QoS
self.camera_sub = self.create_subscription(
    Image,
    '/camera/color/image_raw',
    self.camera_callback,
    sensor_qos
)
```

## DDS Middleware Configuration

ROS 2 uses DDS (Data Distribution Service) as its communication middleware. For humanoid robotics, selecting the right DDS implementation is important:

- **Fast DDS**: Good performance, widely used
- **Cyclone DDS**: Lightweight, good for embedded systems
- **RTI Connext DDS**: Commercial option with advanced features

## ROS 2 Launch Files

Launch files orchestrate multiple nodes for complex humanoid robot behaviors:

```xml
<!-- launch/humanoid_robot.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joint state broadcaster
        Node(
            package='joint_state_broadcaster',
            executable='joint_state_broadcaster',
            name='joint_state_broadcaster'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=['path/to/robot_description.yaml']
        ),

        # IMU driver
        Node(
            package='imu_driver',
            executable='imu_driver',
            name='imu_driver'
        ),

        # Main controller
        Node(
            package='humanoid_controller',
            executable='main_controller',
            name='main_controller'
        )
    ])
```

## ROS 2 for NVIDIA Hardware Integration

### GPU-Accelerated Nodes

ROS 2 nodes can leverage NVIDIA GPUs for AI and computer vision:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import torchvision.transforms as T

class GPUPoweredPerception(Node):
    def __init__(self):
        super().__init__('gpu_perception')

        # Check for GPU availability
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {self.device}')

        # Load model to GPU
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s').to(self.device)

        # Setup ROS 2 interfaces
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10  # Use sensor_qos in production
        )
        self.detection_pub = self.create_publisher(Image, '/detections', 10)

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run inference on GPU
        results = self.model(cv_image)

        # Process results and publish
        annotated_img = results.render()[0]
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
        self.detection_pub.publish(annotated_msg)
```

## ROS 2 Security Features

For humanoid robots operating in human environments, security is paramount:

```python
# Example of setting up ROS 2 security
# In launch file or node:
import os
os.environ['ROS_SECURITY_ENABLE'] = 'true'
os.environ['ROS_SECURITY_STRATEGY'] = 'Enforce'
os.environ['ROS_SECURITY_KEYSTORE'] = '/path/to/keystore'
```

## ROS 2 on Jetson Platforms

### Performance Considerations

When running ROS 2 on Jetson platforms for humanoid robots:

- Use Cyclone DDS for memory efficiency
- Optimize QoS settings for real-time performance
- Consider containerization for isolation
- Monitor resource usage with jtop

## ROS 2 Design Patterns for Humanoid Robotics

### Behavior Tree Integration

For complex humanoid behaviors, integrate with behavior trees:

```python
# Example behavior tree node using ROS 2
import py_trees
import rclpy

class WalkToTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name, target_x, target_y):
        super().__init__(name)
        self.target_x = target_x
        self.target_y = target_y
        self.action_client = None

    def setup(self, **kwargs):
        # Initialize ROS 2 action client
        self.node = kwargs['node']
        self.action_client = ActionClient(
            self.node,
            NavigateToPose,
            'navigate_to_pose'
        )

    def update(self):
        # Send navigation goal
        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = self.target_x
        goal.pose.pose.position.y = self.target_y

        self.action_client.send_goal(goal)

        # Return RUNNING while navigating, SUCCESS when complete
        return py_trees.common.Status.RUNNING
```

## ROS 2 Ecosystem Tools

### ros2cli Tools

Essential tools for debugging and monitoring:

```bash
# List all topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /joint_states

# List all nodes
ros2 node list

# Check node graph
ros2 run rqt_graph rqt_graph
```

## Summary

ROS 2 provides the essential communication architecture for humanoid robotics, enabling distributed processing across multiple nodes. Its QoS settings, security features, and support for real-time systems make it ideal for humanoid robot applications, especially when integrated with NVIDIA's AI platform.

## Next Steps

Now that you understand ROS 2 architecture, continue to [Chapter 4: URDF & Kinematics](./chapter-4-urdf-kinematics.md) to learn about robot description and kinematic modeling.

## Exercises

1. Create a simple ROS 2 publisher and subscriber that communicate sensor data
2. Implement a service that performs a calculation based on robot state
3. Set up a launch file that starts multiple nodes for a simple robot
4. Configure QoS settings for a critical control topic and a sensor topic