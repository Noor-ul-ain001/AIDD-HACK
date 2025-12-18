---
sidebar_position: 4
difficulty: beginner
---

# 2.4: Practical Exercises - Advanced Topics and Node Integration

## Overview

This submodule provides hands-on exercises to integrate the concepts learned about nodes and topics. You'll build a more complex robot simulation system with multiple interconnected nodes communicating through topics.

## Learning Objectives

By the end of this submodule, you will:
- Design and implement a multi-node robot system
- Create custom message types for specific applications
- Configure QoS settings appropriately for different data types
- Implement robust error handling for topic communication
- Use ROS 2 tools to debug and monitor your system

## Exercise 1: Multi-Node Robot System

### Objective
Create a robot system with sensor, controller, and monitor nodes that communicate via topics.

### Steps

1. **Create a new package**:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_system
```

2. **Create a custom message** for robot state (`msg/RobotState.msg`):
```
# RobotState.msg
std_msgs/Header header
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
bool is_moving
```

3. **Create the sensor node** (`robot_system/robot_system/sensor_node.py`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        # Create publishers for different sensor data
        self.laser_pub = self.create_publisher(
            LaserScan, 'laser_scan', 10)
        self.odom_pub = self.create_publisher(
            Float32, 'odometry', 10)
        
        # Create timer for simulated sensor readings
        self.timer = self.create_timer(0.1, self.publish_sensors)  # 10 Hz
        self.get_logger().info('Sensor node started')

    def publish_sensors(self):
        # Publish simulated laser scan
        laser_msg = LaserScan()
        laser_msg.header.stamp = self.get_clock().now().to_msg()
        laser_msg.header.frame_id = 'laser_frame'
        laser_msg.angle_min = -1.57  # -90 degrees
        laser_msg.angle_max = 1.57   # 90 degrees
        laser_msg.angle_increment = 0.1
        laser_msg.range_min = 0.1
        laser_msg.range_max = 10.0
        laser_msg.ranges = [random.uniform(1.0, 5.0) for _ in range(32)]  # 32 range values
        
        self.laser_pub.publish(laser_msg)
        
        # Publish simulated odometry
        odom_msg = Float32()
        odom_msg.data = random.uniform(0.0, 10.0)
        self.odom_pub.publish(odom_msg)
        
        self.get_logger().debug('Published sensor data')

def main(args=None):
    rclpy.init(args=args)
    sensor_node = SensorNode()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()
```

4. **Create the controller node** (`robot_system/robot_system/controller_node.py`):
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Create publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create subscriber for laser data
        self.laser_sub = self.create_subscription(
            LaserScan, 'laser_scan', self.laser_callback, 10)
        
        # Control variables
        self.safe_distance = 1.0  # meters
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 0.3  # rad/s
        
        self.get_logger().info('Controller node started')

    def laser_callback(self, msg):
        # Find the closest obstacle
        min_distance = min(msg.ranges)
        
        # Create twist message
        twist = Twist()
        
        if min_distance < self.safe_distance:
            # Obstacle detected, turn away
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            self.get_logger().info(f'Obstacle detected! Distance: {min_distance:.2f}m, turning')
        else:
            # Path clear, move forward
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.get_logger().info(f'Moving forward, min distance: {min_distance:.2f}m')
        
        # Publish command
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()
```

5. **Create the monitor node** (`robot_system/robot_system/monitor_node.py`):
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        
        # Subscribe to command velocities and odometry
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10)
        self.odom_sub = self.create_subscription(
            Float32, 'odometry', self.odom_callback, 10)
        
        # Track performance metrics
        self.cmd_count = 0
        self.odom_count = 0
        self.last_cmd_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()
        
        self.get_logger().info('Monitor node started')

    def cmd_callback(self, msg):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        self.cmd_count += 1
        self.last_cmd_time = current_time
        
        # Log command info
        self.get_logger().info(
            f'CMD #{self.cmd_count}: Vx={msg.linear.x:.2f}, Wz={msg.angular.z:.2f}, '
            f'Hz={1.0/time_diff if time_diff>0 else 0:.2f}')

    def odom_callback(self, msg):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_odom_time).nanoseconds / 1e9
        
        self.odom_count += 1
        self.last_odom_time = current_time
        
        # Log odometry info
        self.get_logger().info(
            f'ODOM #{self.odom_count}: Pos={msg.data:.2f}, '
            f'Hz={1.0/time_diff if time_diff>0 else 0:.2f}')

def main(args=None):
    rclpy.init(args=args)
    monitor_node = MonitorNode()
    rclpy.spin(monitor_node)
    monitor_node.destroy_node()
    rclpy.shutdown()
```

## Exercise 2: Implementing Custom Messages and QoS

### Objective
Create and use custom messages with appropriate QoS settings for different data types.

### Steps

1. **Update the package.xml** to include message generation:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_system</name>
  <version>0.1.0</version>
  <description>Robot system with multiple nodes</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <buildtool_depend>ament_python</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

2. **Update setup.py** to include the custom message:
```python
from setuptools import setup
from glob import glob
import os

package_name = 'robot_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include any launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Robot system with multiple nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = robot_system.sensor_node:main',
            'controller_node = robot_system.controller_node:main',
            'monitor_node = robot_system.monitor_node:main',
        ],
    },
)
```

3. **Create a launch file** (`robot_system/launch/robot_system_launch.py`):
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_system',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
            # Set parameters if needed
            # parameters=[{'param_name': 'param_value'}]
        ),
        Node(
            package='robot_system',
            executable='controller_node',
            name='controller_node',
            output='screen',
        ),
        Node(
            package='robot_system',
            executable='monitor_node',
            name='monitor_node',
            output='screen',
        ),
    ])
```

4. **Build and run** the system:
```bash
cd ~/ros2_ws
colcon build --packages-select robot_system
source install/setup.bash
ros2 launch robot_system robot_system_launch.py
```

## Exercise 3: Performance Analysis and Optimization

### Objective
Analyze the performance of your system and optimize it.

### Steps

1. **Add performance monitoring to the controller**:
```python
# Add to controller node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Use appropriate QoS for laser data (high-frequency)
        laser_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # More reliable QoS for commands
        cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Create publisher and subscriber with appropriate QoS
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', cmd_qos)
        self.laser_sub = self.create_subscription(
            LaserScan, 'laser_scan', self.laser_callback, laser_qos)
        
        # Performance tracking
        self.processing_times = []
        self.get_logger().info('Controller node started with QoS optimization')

    def laser_callback(self, msg):
        start_time = self.get_clock().now()
        
        # Your existing laser processing code here
        min_distance = min(msg.ranges)
        twist = Twist()
        
        if min_distance < self.safe_distance:
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        
        self.cmd_pub.publish(twist)
        
        # Track processing time
        end_time = self.get_clock().now()
        processing_time = (end_time - start_time).nanoseconds / 1e6  # milliseconds
        self.processing_times.append(processing_time)
        
        if len(self.processing_times) > 100:
            self.processing_times = self.processing_times[-100:]
        
        avg_time = sum(self.processing_times) / len(self.processing_times)
        self.get_logger().info(f'Processing time: {processing_time:.2f}ms, Avg: {avg_time:.2f}ms')
```

2. **Monitor system performance**:
```bash
# Monitor topics
ros2 topic hz /laser_scan
ros2 topic hz /cmd_vel

# Monitor nodes
ros2 run top top

# Monitor with command-line tools
ros2 topic bw
```

## Exercise 4: Error Handling and Recovery

### Objective
Implement robust error handling in your system.

### Steps

1. **Enhanced sensor node with error handling**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random
from rclpy.qos import QoSProfile

class RobustSensorNode(Node):
    def __init__(self):
        super().__init__('robust_sensor_node')
        
        # Create publishers with different QoS for different data
        reliable_qos = QoSProfile(depth=10, reliability=1)  # RELIABLE
        best_effort_qos = QoSProfile(depth=5, reliability=0)  # BEST_EFFORT
        
        self.laser_pub = self.create_publisher(
            LaserScan, 'laser_scan', reliable_qos)
        
        # Timer with error handling
        self.timer = self.create_timer(0.1, self.publish_sensors)
        self.error_count = 0
        self.get_logger().info('Robust sensor node started')

    def publish_sensors(self):
        try:
            # Simulate occasional sensor errors
            if random.random() < 0.05:  # 5% error rate
                self.get_logger().warn('Simulated sensor error')
                raise Exception("Sensor reading error")
            
            # Create and publish laser scan message
            laser_msg = LaserScan()
            laser_msg.header.stamp = self.get_clock().now().to_msg()
            laser_msg.header.frame_id = 'laser_frame'
            laser_msg.angle_min = -1.57
            laser_msg.angle_max = 1.57
            laser_msg.angle_increment = 0.1
            laser_msg.range_min = 0.1
            laser_msg.range_max = 10.0
            laser_msg.ranges = [random.uniform(1.0, 5.0) for _ in range(32)]
            
            self.laser_pub.publish(laser_msg)
            if self.error_count > 0:
                self.error_count = 0  # Reset error counter on success
                self.get_logger().info('Sensor recovered from error')
                
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Sensor error #{self.error_count}: {e}')
            
            # Implement recovery strategy after N consecutive errors
            if self.error_count >= 5:
                self.get_logger().error('Too many consecutive errors, initiating recovery...')
                self.attempt_recovery()

    def attempt_recovery(self):
        # Simulate recovery attempt
        self.get_logger().info('Attempting sensor recovery...')
        # In a real system, this might reinitialize hardware, etc.
        self.error_count = 0
        self.get_logger().info('Recovery attempt completed')

def main(args=None):
    rclpy.init(args=args)
    sensor_node = RobustSensorNode()
    
    try:
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        sensor_node.get_logger().info('Interrupted by user')
    finally:
        sensor_node.destroy_node()
        rclpy.shutdown()
```

## Exercise 5: System Integration Testing

### Objective
Test the complete system and validate communication between nodes.

### Steps

1. **Create a test script** (`robot_system/test_system.py`):
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import time

class SystemTester(Node):
    def __init__(self):
        super().__init__('system_tester')
        
        # Subscribe to all relevant topics
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, 'laser_scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(
            Float32, 'odometry', self.odom_callback, 10)
        
        # Track message counts
        self.cmd_count = 0
        self.laser_count = 0
        self.odom_count = 0
        
        # Test timer
        self.test_timer = self.create_timer(5.0, self.run_test)
        self.test_step = 0
        
        self.get_logger().info('System tester started')

    def cmd_callback(self, msg):
        self.cmd_count += 1

    def laser_callback(self, msg):
        self.laser_count += 1

    def odom_callback(self, msg):
        self.odom_count += 1

    def run_test(self):
        self.test_step += 1
        
        if self.test_step == 1:
            self.get_logger().info('Starting system test...')
            self.get_logger().info('Initial counts - Cmd: {}, Laser: {}, Odom: {}'.format(
                self.cmd_count, self.laser_count, self.odom_count))
        elif self.test_step == 2:
            current_counts = f'Cmd: {self.cmd_count}, Laser: {self.laser_count}, Odom: {self.odom_count}'
            self.get_logger().info(f'Counts after 5s: {current_counts}')
            
            # Verify that messages are flowing
            if all(count > 0 for count in [self.cmd_count, self.laser_count, self.odom_count]):
                self.get_logger().info('✓ All topics are active')
            else:
                self.get_logger().error('✗ Some topics are inactive')
        elif self.test_step == 3:
            final_counts = f'Cmd: {self.cmd_count}, Laser: {self.laser_count}, Odom: {self.odom_count}'
            self.get_logger().info(f'Final counts: {final_counts}')
            self.get_logger().info('System test completed')
            self.test_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    tester = SystemTester()
    rclpy.spin(tester)
    tester.destroy_node()
    rclpy.shutdown()
```

2. **Add test entry point to setup.py**:
```python
entry_points={
    'console_scripts': [
        'sensor_node = robot_system.sensor_node:main',
        'controller_node = robot_system.controller_node:main',
        'monitor_node = robot_system.monitor_node:main',
        'system_tester = robot_system.test_system:main',  # Add this line
    ],
},
```

3. **Run the complete test**:
```bash
# Terminal 1: Start the robot system
cd ~/ros2_ws
colcon build --packages-select robot_system
source install/setup.bash
ros2 run robot_system sensor_node & ros2 run robot_system controller_node & ros2 run robot_system monitor_node

# Terminal 2: Run the system tester
source install/setup.bash
ros2 run robot_system system_tester
```

## Summary

These practical exercises helped you build a complete multi-node robot system using ROS 2 concepts including:
- Node creation and management
- Topic communication patterns
- Custom message types
- QoS configuration
- Error handling and recovery
- System testing and validation

## Additional Challenges

1. **Enhance the system**: Add more sensor types (IMU, camera) with appropriate QoS settings
2. **Implement navigation**: Create a path-following controller
3. **Add simulation**: Use Gazebo to test with a simulated robot
4. **Create a dashboard**: Develop a simple GUI to monitor system status
5. **Implement logging**: Add comprehensive logging for debugging

This completes the submodules for Week 2 of Module 1 on Nodes and Topics in ROS 2.