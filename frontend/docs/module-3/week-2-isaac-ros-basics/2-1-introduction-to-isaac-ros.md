---
sidebar_position: 1
difficulty: intermediate
---

# Week 2: Isaac ROS Basics

## Overview

This week introduces Isaac ROS, NVIDIA's robotics platform that combines the Robot Operating System (ROS) with NVIDIA's GPU-accelerated libraries for AI and perception.

## Learning Objectives

By the end of this week, you will:
- Understand the fundamentals of Isaac ROS
- Set up Isaac ROS for your robotic applications
- Implement perception pipelines using Isaac ROS
- Integrate Isaac ROS with existing ROS 2 systems

## Introduction to Isaac ROS

Isaac ROS is NVIDIA's accelerated perception and navigation stack built for ROS 2. It provides optimized, hardware-accelerated packages that leverage NVIDIA GPUs to accelerate perception, mapping, and navigation tasks.

### Key Features of Isaac ROS

1. **Hardware Acceleration**: GPU-accelerated algorithms for real-time performance
2. **Perception Pipeline**: Optimized computer vision and deep learning nodes
3. **Integration**: Seamless integration with existing ROS 2 ecosystem
4. **Docker Support**: Easy deployment with containerized packages

### Isaac ROS Architecture

```
Hardware Layer (NVIDIA Jetson/RTX)
        |
GPU Acceleration Layer
        |
Isaac ROS Packages
        |
ROS 2 Middleware
        |
Applications
```

## Installing Isaac ROS

### System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | NVIDIA GPU with CUDA | NVIDIA RTX 3060 or better |
| CUDA | 11.8+ | 12.0+ |
| OS | Ubuntu 22.04 | Ubuntu 22.04 LTS |
| RAM | 8 GB | 16+ GB |
| Storage | 10 GB | 20+ GB |

### Installation Method 1: Debian Packages

```bash
# Add Isaac ROS repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://repos.packages.nvidia.com/keys/all-keys.gpg | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/nvidia-archive-keyring.gpg] https://repos.packages.nvidia.com/isaac/$(lsb_release -cs)/ all main" | sudo tee /etc/apt/sources.list.d/nvidia-isaac-repos-$(lsb_release -cs).list
sudo apt update

# Install Isaac ROS packages
sudo apt install nvidia-isaac-ros-gem
```

### Installation Method 2: Docker

```bash
# Pull Isaac ROS Docker image
docker pull nvcr.io/nvidia/isaac-ros:latest

# Run Isaac ROS container
docker run --gpus all -it --rm --net=host nvcr.io/nvidia/isaac-ros:latest
```

## Core Isaac ROS Packages

### Isaac ROS AprilTag

Detect and estimate poses of AprilTag markers:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        
        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for detections
        self.detection_pub = self.create_publisher(
            AprilTagDetectionArray,
            '/apriltag_detections',
            10
        )
    
    def image_callback(self, msg):
        # Process image for AprilTag detection
        # (Actual processing handled by Isaac ROS node)
        pass
```

### Isaac ROS Visual SLAM

Real-time visual SLAM for localization and mapping:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class VisualSLAMNode(Node):
    def __init__(self):
        super().__init__('visual_slam')
        
        # Subscribe to stereo camera or RGB-D data
        self.left_image_sub = self.create_subscription(
            Image,
            '/camera/left/image_raw',
            self.left_image_callback,
            10
        )
        
        self.right_image_sub = self.create_subscription(
            Image,
            '/camera/right/image_raw',
            self.right_image_callback,
            10
        )
        
        # Publishers for pose and map
        self.pose_pub = self.create_publisher(PoseStamped, '/visual_slam/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/visual_slam/odometry', 10)
```

## Launching Isaac ROS Nodes

### Example Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Isaac ROS AprilTag container
    apriltag_container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag',
                parameters=[{
                    'family': 'tag36h11',
                    'max_tags': 10,
                    'tag_size': 0.166
                }]
            )
        ],
        output='screen',
    )

    return LaunchDescription([apriltag_container])
```

## Isaac ROS Perception Pipelines

### Image Preprocessing Pipeline

```python
# Isaac ROS provides optimized image preprocessing nodes
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_image_preprocessing_pipeline():
    image_processing_container = ComposableNodeContainer(
        name='image_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Image resize node
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                name='resize_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image', 'camera/image_raw'),
                    ('camera_info', 'camera/camera_info'),
                    ('resized/image', 'camera/image_resized'),
                    ('resized/camera_info', 'camera/camera_info_resized'),
                ],
            ),
            
            # Image format converter
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::FormatConverterNode',
                name='format_converter_node',
                remappings=[
                    ('image', 'camera/image_resized'),
                    ('camera_info', 'camera/camera_info_resized'),
                    ('output/image', 'camera/image_formatted'),
                    ('output/camera_info', 'camera/camera_info_formatted'),
                ],
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([image_processing_container])
```

## Integration with ROS 2 Ecosystem

### Connecting Isaac ROS to Standard ROS 2 Nodes

Isaac ROS nodes publish standard ROS 2 message types, making them compatible with the broader ROS ecosystem:

```python
# Standard ROS 2 node consuming Isaac ROS output
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

class IsaacIntegrationNode(Node):
    def __init__(self):
        super().__init__('isaac_integration_node')
        
        # Subscribe to Isaac ROS pose output
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/pose',
            self.pose_callback,
            10
        )
        
        # Subscribe to Isaac ROS processed image
        self.processed_image_sub = self.create_subscription(
            Image,
            '/isaac_ros/image_processed',
            self.image_callback,
            10
        )
    
    def pose_callback(self, msg):
        # Use Isaac ROS pose in standard ROS 2 application
        self.get_logger().info(f'Position: x={msg.pose.position.x}, y={msg.pose.position.y}')
    
    def image_callback(self, msg):
        # Process Isaac ROS accelerated image
        self.get_logger().info(f'Received processed image: {msg.width}x{msg.height}')
```

## Practical Exercise

This week's exercise involves setting up an Isaac ROS perception pipeline:

1. Install Isaac ROS on your development platform
2. Configure camera input for Isaac ROS processing
3. Implement an AprilTag detection pipeline
4. Integrate the pipeline with a navigation system

## Summary

This week introduced Isaac ROS, NVIDIA's accelerated robotics platform for perception and navigation. You've learned about installation, core packages, and integration with ROS 2. Next week, we'll explore advanced Isaac Sim techniques.