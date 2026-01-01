---
sidebar_position: 1
difficulty: beginner
---

# 4.1: Understanding ROS 2 TF2 (Transform Library)

## Overview

This submodule introduces TF2 (Transform Library), ROS 2's system for tracking coordinate frame relationships over time. TF2 is essential for robotics applications that need to transform data between different coordinate frames, such as sensor data, robot poses, and map coordinates.

## Learning Objectives

By the end of this submodule, you will:
- Understand the concept of coordinate frames and transformations
- Learn how TF2 works in ROS 2
- Create and broadcast transforms using TF2
- Listen to and use transforms from TF2
- Apply TF2 in practical robotic scenarios
- Debug common TF2 issues

## Spatial Relationships in Robotics

### Coordinate Frames

In robotics, we use coordinate frames to provide context for spatial measurements. Common frames include:

- **Base frame** (`base_link`, `base_footprint`): Robot's center
- **Sensor frames** (`camera_frame`, `laser_frame`): Sensor mounting positions
- **World/mapping frame** (`map`, `odom`): Reference for position tracking
- **End-effector frame** (`gripper`, `tool0`): Robot's working point

### Why TF2 is Needed

When a robot has multiple sensors and moving parts, we need a way to:
- Relate data from different sensors
- Track the position of robot parts over time
- Transform points between coordinate systems
- Maintain a consistent spatial understanding

## TF2 Architecture

### Key Components

1. **Transform Buffer**: Stores all frame relationships
2. **Transform Listener**: Subscribes to transform data
3. **Transform Broadcaster**: Publishes new transform data
4. **TF2 Message Types**: `tf2_msgs/TFMessage` and related types

### TF2 vs TF

TF2 is the second generation of the transform library with improvements:
- Better performance
- More intuitive API
- Support for custom datatypes
- Cleaner separation of concerns

## Broadcasting Transforms

### Static Transforms

Static transforms are constant relationships between frames (e.g., sensor mounted on robot).

**Python Implementation:**
```python
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')
        
        # Create a StaticTransformBroadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish the static transform
        self.publish_static_transform()
    
    def publish_static_transform(self):
        # Create a transform
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'
        
        # Set translation (x, y, z)
        t.transform.translation.x = 0.1  # 10 cm forward
        t.transform.translation.y = 0.0  # No lateral offset
        t.transform.translation.z = 0.2  # 20 cm up
        
        # Set rotation (as quaternion)
        # For a simple 90-degree rotation around Z axis
        q = tf_transformations.quaternion_from_euler(0, 0, 1.5708)  # 90 degrees in radians
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Send the transform
        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info('Published static transform: base_link -> laser_frame')

def main(args=None):
    rclpy.init(args=args)
    node = StaticFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Dynamic Transforms

Dynamic transforms change over time (e.g., rotating sensor, moving robot parts).

**Python Implementation:**
```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import math

class DynamicFramePublisher(Node):
    def __init__(self):
        super().__init__('dynamic_frame_publisher')
        
        # Create a TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create a timer to periodically broadcast transforms
        self.timer = self.create_timer(0.1, self.broadcast_transform)  # 10 Hz
        self.time_step = 0.0
    
    def broadcast_transform(self):
        # Create a transform
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'rotating_sensor'
        
        # Make the transform oscillate for demonstration
        # Translation moves in a circle
        radius = 0.3  # 30 cm radius
        t.transform.translation.x = radius * math.cos(self.time_step)
        t.transform.translation.y = radius * math.sin(self.time_step)
        t.transform.translation.z = 0.5  # Fixed height
        
        # Rotation changes over time
        q = tf_transformations.quaternion_from_euler(0, 0, self.time_step)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)
        
        self.time_step += 0.1  # Increment for next iteration
        
        if self.time_step % 10 < 0.1:  # Log every 10 seconds
            self.get_logger().info(f'Published dynamic transform: base_link -> rotating_sensor')

def main(args=None):
    rclpy.init(args=args)
    node = DynamicFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Listening to Transforms

### Basic Transform Listener

**Python Implementation:**
```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PointStamped

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')
        
        # Create a transform buffer
        self.tf_buffer = Buffer()
        
        # Create a transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create a timer to periodically look up transforms
        self.timer = self.create_timer(1.0, self.lookup_transform)
    
    def lookup_transform(self):
        try:
            # Look up the transform from 'base_link' to 'laser_frame'
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link',  # Target frame
                'laser_frame',  # Source frame
                now)  # Time (use 'now' for most recent)
            
            # Log the transform
            self.get_logger().info(
                f'Translation: x={trans.transform.translation.x:.3f}, '
                f'y={trans.transform.translation.y:.3f}, '
                f'z={trans.transform.translation.z:.3f}')
            self.get_logger().info(
                f'Rotation: x={trans.transform.rotation.x:.3f}, '
                f'y={trans.transform.rotation.y:.3f}, '
                f'z={trans.transform.rotation.z:.3f}, '
                f'w={trans.transform.rotation.w:.3f}')
        
        except Exception as e:
            self.get_logger().error(f'Could not get transform: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Transform with Time Lookup

**Python Implementation:**
```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PointStamped, TransformStamped
from builtin_interfaces.msg import Time
import time

class TimedFrameListener(Node):
    def __init__(self):
        super().__init__('timed_frame_listener')
        
        # Create a transform buffer
        self.tf_buffer = Buffer()
        
        # Create a transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create a publisher for transformed points
        self.point_pub = self.create_publisher(PointStamped, 'transformed_point', 10)
        
        # Timer to demonstrate timed transforms
        self.timer = self.create_timer(2.0, self.lookup_timed_transform)
    
    def lookup_timed_transform(self):
        try:
            # Create a point in the laser frame
            point_in_laser = PointStamped()
            point_in_laser.header.frame_id = 'laser_frame'
            point_in_laser.header.stamp = self.get_clock().now().to_msg()
            point_in_laser.point.x = 1.0  # 1 meter in front of laser
            point_in_laser.point.y = 0.0
            point_in_laser.point.z = 0.0
            
            # Transform the point to base_link frame
            point_in_base = self.tf_buffer.transform(
                point_in_laser,    # Input point
                'base_link',       # Target frame
                timeout=rclpy.duration.Duration(seconds=1.0))  # Timeout
            
            # Log the result
            self.get_logger().info(
                f'Transformed point: ({point_in_base.point.x:.3f}, '
                f'{point_in_base.point.y:.3f}, {point_in_base.point.z:.3f}) '
                f'in base_link frame')
            
            # Publish the transformed point
            self.point_pub.publish(point_in_base)
        
        except Exception as e:
            self.get_logger().error(f'Transform failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TimedFrameListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Transforming Data

### Transforming Points, Vectors, and Poses

**Python Implementation:**
```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PointStamped, Vector3Stamped, PoseStamped
from sensor_msgs.msg import LaserScan

class DataTransformer(Node):
    def __init__(self):
        super().__init__('data_transformer')
        
        # Create a transform buffer
        self.tf_buffer = Buffer()
        
        # Create a transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to laser scan
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # Publisher for transformed data
        self.transformed_scan_pub = self.create_publisher(
            LaserScan, 'scan_in_base_link', 10)
    
    def scan_callback(self, msg):
        try:
            # Transform the entire scan to base_link frame
            # This is a simplified example - in practice, you'd transform individual points
            transform = self.tf_buffer.lookup_transform(
                'base_link',      # Target frame
                msg.header.frame_id,  # Source frame (usually laser_frame)
                msg.header.stamp,     # Time of the scan
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Log that we have the transform
            self.get_logger().info(
                f'Got transform from {msg.header.frame_id} to base_link')
        
        except Exception as e:
            self.get_logger().error(f'Could not transform scan: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = DataTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## TF2 in C++

### Static Transform Broadcaster (C++)
```cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class StaticFramePublisher : public rclcpp::Node
{
public:
    StaticFramePublisher() : Node("static_frame_publisher")
    {
        tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // Publish the static transform
        publish_static_transform();
    }

private:
    void publish_static_transform()
    {
        geometry_msgs::msg::TransformStamped t;
        
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "base_link";
        t.child_frame_id = "laser_frame";
        
        // Set translation
        t.transform.translation.x = 0.1;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.2;
        
        // Set rotation (90 degrees around Z axis)
        tf2::Quaternion q;
        q.setRPY(0, 0, 1.5708);  // roll, pitch, yaw
        
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        tf_publisher_->sendTransform(t);
        RCLCPP_INFO(this->get_logger(), "Published static transform: base_link -> laser_frame");
    }
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticFramePublisher>());
    rclcpp::shutdown();
    return 0;
}
```

### Transform Listener (C++)
```cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class FrameListener : public rclcpp::Node
{
public:
    FrameListener() : Node("frame_listener")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create a timer to periodically look up transforms
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&FrameListener::lookup_transform, this));
    }

private:
    void lookup_transform()
    {
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "base_link",    // Target frame
                "laser_frame",  // Source frame
                tf2::TimePointZero);  // Most recent available
            
            RCLCPP_INFO(this->get_logger(),
                "Translation: x=%f, y=%f, z=%f",
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z);
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcl::TimerBase::SharedPtr timer_;
};
```

## TF2 Best Practices

### Frame Naming Conventions

- Use consistent naming: `sensor_name_frame` (e.g., `camera_frame`, `lidar_frame`)
- Use semantic names: `map`, `odom`, `base_link`, `base_footprint`
- Follow REP-105 conventions for standard frame names

### Performance Considerations

```python
# Efficient transform lookup
class EfficientTransformer(Node):
    def __init__(self):
        super().__init__('efficient_transformer')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Cache transforms for frequently accessed frames
        self.cached_transforms = {}
    
    def get_cached_transform(self, target_frame, source_frame):
        cache_key = f"{target_frame}_{source_frame}"
        
        # Try to get from cache first
        if cache_key in self.cached_transforms:
            return self.cached_transforms[cache_key]
        
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            # Cache the transform (with appropriate invalidation strategy)
            self.cached_transforms[cache_key] = transform
            return transform
        except Exception as e:
            self.get_logger().error(f'Transform lookup failed: {e}')
            return None
```

### Transform Error Handling

```python
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class RobustTransformer(Node):
    def __init__(self):
        super().__init__('robust_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def safe_lookup_transform(self, target_frame, source_frame, time=None):
        try:
            if time is None:
                time = rclpy.time.Time()
            
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, time)
            return transform, True  # Success
        except LookupException:
            self.get_logger().warn(f'Frame {source_frame} does not exist')
            return None, False
        except ConnectivityException:
            self.get_logger().warn(f'No connection from {source_frame} to {target_frame}')
            return None, False
        except ExtrapolationException:
            self.get_logger().warn(f'Transform would require extrapolation')
            return None, False
        except Exception as e:
            self.get_logger().error(f'Unexpected error in transform lookup: {e}')
            return None, False
```

## TF2 Tools and Debugging

### Command-Line Tools

```bash
# View the TF tree
ros2 run tf2_tools view_frames

# Echo a specific transform
ros2 run tf2_ros tf2_echo base_link laser_frame

# View TF tree as image (creates frames.pdf)
# After running view_frames
evince frames.pdf  # or your system's PDF viewer
```

### TF2 Monitor

```bash
# Monitor TF quality
ros2 run tf2_ros tf2_monitor
```

## Common TF2 Issues and Solutions

### 1. Transform Not Available Error

```python
# Problem: Getting "could not find transform" errors
# Solution: Wait for transforms to be available

import time
from rclpy.duration import Duration

class WaitForTransform(Node):
    def __init__(self):
        super().__init__('wait_for_transform')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def lookup_with_wait(self):
        try:
            # Wait for transform to become available
            self.tf_buffer.wait_for_transform(
                'base_link', 'laser_frame', 
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
            
            # Now do the lookup
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'laser_frame', rclpy.time.Time())
            
            return transform
        except Exception as e:
            self.get_logger().error(f'Failed to get transform even after waiting: {e}')
            return None
```

### 2. Timing Issues

```python
# Problem: Transform at wrong time
# Solution: Use appropriate time synchronization

class TimeSyncedTransformer(Node):
    def __init__(self):
        super().__init__('time_synced_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def transform_with_sensor_time(self, sensor_msg):
        try:
            # Use the time from the sensor message for transform
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                sensor_msg.header.frame_id,
                sensor_msg.header.stamp,
                timeout=Duration(seconds=0.1))
            
            return transform
        except Exception as e:
            self.get_logger().error(f'Time-synced transform failed: {e}')
            return None
```

## Summary

This submodule covered the fundamentals of ROS 2 TF2 (Transform Library):

- Coordinate frames and transformations in robotics
- Broadcasting static and dynamic transforms
- Listening to and using transforms
- Transforming data between coordinate frames
- Best practices and common debugging techniques

TF2 is essential for any robotics application that needs to understand spatial relationships between different parts of the robot and its environment. In the next submodule, we'll explore URDF (Unified Robot Description Format), which works closely with TF2 to describe robot structure.