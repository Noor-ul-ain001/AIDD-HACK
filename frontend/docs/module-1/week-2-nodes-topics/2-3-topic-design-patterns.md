---
sidebar_position: 3
difficulty: advanced
---

# 2.3: Topic Design Patterns and Best Practices

## Overview

This submodule covers advanced topic design patterns and best practices to create efficient, maintainable, and robust ROS 2 applications.

## Learning Objectives

By the end of this submodule, you will:
- Apply advanced design patterns for topics
- Understand data serialization and bandwidth considerations
- Implement proper error handling and fault tolerance
- Design scalable topic architectures
- Use advanced QoS configurations effectively

## Topic Design Patterns

### 1. Event-Driven Pattern

Use topics to notify other nodes of system events:

```python
# Event publisher
from std_msgs.msg import String

class EventPublisher(Node):
    def __init__(self):
        super().__init__('event_publisher')
        self.event_pub = self.create_publisher(String, 'system_events', 10)
    
    def publish_event(self, event_type, details=""):
        msg = String()
        msg.data = f"{event_type}:{details}"
        self.event_pub.publish(msg)

# Usage example
event_publisher = EventPublisher()
event_publisher.publish_event("SENSOR_FAILURE", "Lidar sensor disconnected")
```

### 2. Data Aggregation Pattern

Combine multiple data sources into a single topic:

```python
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from my_robot_msgs.msg import RobotStatus  # Custom aggregated message

class DataAggregator(Node):
    def __init__(self):
        super().__init__('data_aggregator')
        
        # Subscriptions to multiple data sources
        self.lidar_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # Aggregated output
        self.status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        
        self.last_lidar = None
        self.last_imu = None
        self.last_cmd = None
    
    def lidar_callback(self, msg):
        self.last_lidar = msg
        self.publish_aggregated_status()
    
    def imu_callback(self, msg):
        self.last_imu = msg
        self.publish_aggregated_status()
    
    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.publish_aggregated_status()
    
    def publish_aggregated_status(self):
        if all([self.last_lidar, self.last_imu, self.last_cmd]):
            msg = RobotStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.lidar_data = self.last_lidar.ranges[:10]  # Simplified
            msg.imu_data.orientation = self.last_imu.orientation
            msg.command.linear = self.last_cmd.linear
            msg.command.angular = self.last_cmd.angular
            
            self.status_pub.publish(msg)
```

### 3. Hierarchical Topic Pattern

Organize topics in a hierarchy based on robot components:

```python
# Good topic naming structure
# /robot_name/system/component/data_type
# Examples:
# /turtle1/sensors/lidar/scan
# /turtle1/motors/wheel_front_left/velocity
# /turtle1/control/cmd_vel
# /arm_1/joint_states/position
# /arm_1/end_effector/pose
```

### 4. State Synchronization Pattern

Use a single topic to maintain system state across nodes:

```python
from my_robot_msgs.msg import SystemState

class StateSynchronizer(Node):
    def __init__(self):
        super().__init__('state_synchronizer')
        
        # State publisher
        self.state_pub = self.create_publisher(SystemState, 'system_state', 1)
        
        # State subscription (for monitoring)
        self.state_sub = self.create_subscription(
            SystemState, 'system_state', self.state_callback, 1)
        
        # Initialize system state
        self.system_state = SystemState()
    
    def update_and_publish_state(self, component, value):
        # Update specific component in system state
        if component == "battery":
            self.system_state.battery_level = value
        elif component == "position":
            self.system_state.position = value
        
        # Publish entire state (synchronized)
        self.system_state.header.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(self.system_state)
    
    def state_callback(self, msg):
        # Update local state representation
        self.system_state = msg
```

## Data Serialization and Bandwidth Optimization

### Efficient Message Design

```python
# Good: Compact message design
# CompactSensorData.msg
float32[3] position  # Instead of separate x, y, z
uint8 sensor_type   # Use enums instead of strings
float32 confidence  # Single confidence value instead of complex structure

# Avoid: Excessive data in messages
# Don't send entire images on regular topics (use image_transport)
# Don't send large arrays when only a few values are needed
```

### Data Compression and Downsampling

```python
from sensor_msgs.msg import PointCloud2
import numpy as np

class DataCompressor(Node):
    def __init__(self):
        super().__init__('data_compressor')
        
        self.raw_sub = self.create_subscription(
            PointCloud2, 'raw_pointcloud', self.raw_callback, 1)
        self.compressed_pub = self.create_publisher(
            PointCloud2, 'compressed_pointcloud', 1)
    
    def raw_callback(self, msg):
        # Downsample or filter data before publishing
        if self.should_compress():
            compressed_msg = self.downsample_pointcloud(msg)
            self.compressed_pub.publish(compressed_msg)
        else:
            self.compressed_pub.publish(msg)
    
    def downsample_pointcloud(self, cloud_msg):
        # Implementation to reduce point cloud density
        # This is a simplified example
        return cloud_msg  # Placeholder
```

### Rate Limiting

```python
from rclpy.qos import QoSProfile

class RateLimitedPublisher(Node):
    def __init__(self):
        super().__init__('rate_limited_publisher')
        
        # Create timer for rate limiting
        self.timer = self.create_timer(0.1, self.publish_callback)  # 10 Hz
        self.publisher = self.create_publisher(String, 'rate_limited_topic', 
                                             QoSProfile(depth=1))
        
        self.counter = 0
    
    def publish_callback(self):
        msg = String()
        msg.data = f"Message {self.counter}"
        self.publisher.publish(msg)
        self.counter += 1
```

## Advanced QoS Configurations

### Real-time Performance QoS

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import LivelinessPolicy

# For real-time applications
realtime_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
    liveliness=LivelinessPolicy.AUTOMATIC,
    deadline=Duration(seconds=1),
    lifespan=Duration(seconds=30)
)
```

### Reliable Communication QoS

```python
# For mission-critical applications
critical_qos = QoSProfile(
    depth=100,  # Larger buffer for important data
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_ALL,  # Keep all messages
    durability=DurabilityPolicy.TRANSIENT_LOCAL  # Persist messages
)
```

### Best-effort Communication QoS

```python
# For high-frequency, non-critical data
best_effort_qos = QoSProfile(
    depth=5,  # Small buffer for frequent updates
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE
)
```

## Error Handling and Fault Tolerance

### Publisher with Error Recovery

```python
from rclpy.qos import QoSProfile
from std_msgs.msg import Header

class RobustPublisher(Node):
    def __init__(self):
        super().__init__('robust_publisher')
        
        # Use appropriate QoS for reliability
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(String, 'robust_topic', qos_profile)
        
        self.fallback_mode = False
        self.recovery_counter = 0
        
        # Timer to attempt recovery
        self.recovery_timer = self.create_timer(5.0, self.recovery_callback)
    
    def publish_with_fallback(self, msg_data):
        try:
            msg = String()
            msg.data = msg_data
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Publish failed: {e}')
            self.fallback_mode = True
            self.handle_publish_failure(msg_data)
    
    def handle_publish_failure(self, msg_data):
        # Log error for later analysis
        self.get_logger().error(f'Failed to publish: {msg_data}')
        # Implement appropriate fallback behavior
        # (e.g., store in local buffer, send to alternative topic, etc.)
    
    def recovery_callback(self):
        if self.fallback_mode:
            self.recovery_counter += 1
            if self.recovery_counter >= 3:  # Try recovery after 3 attempts
                self.attempt_recovery()
    
    def attempt_recovery(self):
        self.get_logger().info('Attempting to recover publisher...')
        try:
            # Reinitialize publisher if needed
            qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.publisher = self.create_publisher(String, 'robust_topic', qos_profile)
            self.fallback_mode = False
            self.recovery_counter = 0
            self.get_logger().info('Publisher recovery successful')
        except Exception as e:
            self.get_logger().error(f'Publisher recovery failed: {e}')
```

### Subscriber with Validation

```python
from std_msgs.msg import Float32

class ValidatingSubscriber(Node):
    def __init__(self):
        super().__init__('validating_subscriber')
        
        self.subscription = self.create_subscription(
            Float32, 'sensor_value', self.sensor_callback, 10)
        
        # Store previous values for change detection
        self.previous_values = []
        self.max_change_rate = 10.0  # Max acceptable change per second
        self.last_time = self.get_clock().now()
    
    def sensor_callback(self, msg):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_time).nanoseconds / 1e9  # seconds
        
        # Validate message value
        if not self.is_value_valid(msg.data):
            self.get_logger().warn(f'Invalid sensor value received: {msg.data}')
            return
        
        # Check for excessive changes
        if len(self.previous_values) > 0 and time_diff > 0:
            rate_of_change = abs(msg.data - self.previous_values[-1]) / time_diff
            if rate_of_change > self.max_change_rate:
                self.get_logger().warn(f'Excessive rate of change detected: {rate_of_change}')
        
        # Store value and update time
        self.previous_values.append(msg.data)
        self.last_time = current_time
        
        # Process valid message
        self.process_sensor_value(msg.data)
    
    def is_value_valid(self, value):
        # Example validation: check for reasonable sensor range
        return -100.0 <= value <= 100.0
    
    def process_sensor_value(self, value):
        # Process the validated sensor value
        self.get_logger().info(f'Processing sensor value: {value}')
```

## Topic Monitoring and Diagnostics

### Diagnostic Publisher for Topics

```python
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class TopicDiagnostics(Node):
    def __init__(self):
        super().__init__('topic_diagnostics')
        
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self.status_timer = self.create_timer(1.0, self.publish_diagnostics)
        
        # Track topic statistics
        self.topic_stats = {
            'topic_name': {
                'message_count': 0,
                'last_message_time': None,
                'avg_frequency': 0.0
            }
        }
    
    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        for topic_name, stats in self.topic_stats.items():
            status = DiagnosticStatus()
            status.name = f'Topic {topic_name}'
            status.hardware_id = 'ROS2'
            
            # Determine status level based on statistics
            if stats['message_count'] == 0:
                status.level = DiagnosticStatus.ERROR
                status.message = 'No messages received'
            elif stats['avg_frequency'] < 0.1:  # Less than 0.1 Hz
                status.level = DiagnosticStatus.WARN
                status.message = f'Low frequency: {stats["avg_frequency"]:.2f} Hz'
            else:
                status.level = DiagnosticStatus.OK
                status.message = f'Active: {stats["avg_frequency"]:.2f} Hz'
            
            # Add key-value pairs
            status.values = [
                KeyValue(key='Message Count', value=str(stats['message_count'])),
                KeyValue(key='Frequency', value=f'{stats["avg_frequency"]:.2f} Hz')
            ]
            
            diag_array.status.append(status)
        
        self.diag_pub.publish(diag_array)
```

## Performance Considerations

### Memory Management for High-Frequency Topics

```python
from std_msgs.msg import Float32MultiArray
import weakref

class EfficientHighFreqSubscriber(Node):
    def __init__(self):
        super().__init__('efficient_subscriber')
        
        # Use appropriate QoS for high-frequency data
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Float32MultiArray, 'high_freq_data', self.data_callback, qos)
        
        # Limit stored references to prevent memory leaks
        self.processed_messages = []
        self.max_stored_messages = 100
    
    def data_callback(self, msg):
        # Process message immediately
        processed_data = self.process_message(msg)
        
        # Store limited history to prevent memory buildup
        self.processed_messages.append(processed_data)
        if len(self.processed_messages) > self.max_stored_messages:
            self.processed_messages = self.processed_messages[-50:]  # Keep last 50
        
        # Don't store references to msg if not needed
        del msg  # Just a reference, doesn't free the actual message
```

## Summary

This submodule covered advanced topic design patterns, bandwidth optimization, robust error handling, and performance considerations for ROS 2 topics. In the final submodule for Week 2, we'll put these concepts into practice with more complex examples.