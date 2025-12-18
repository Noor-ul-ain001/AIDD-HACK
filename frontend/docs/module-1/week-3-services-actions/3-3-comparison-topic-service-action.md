---
sidebar_position: 3
difficulty: beginner
---

# 3.3: Comparison of Communication Patterns - Topic vs Service vs Action

## Overview

This submodule compares the three primary communication patterns in ROS 2: topics, services, and actions. We'll examine their characteristics, appropriate use cases, and how to choose between them for different scenarios.

## Learning Objectives

By the end of this submodule, you will:
- Compare the characteristics of topics, services, and actions
- Identify appropriate use cases for each communication pattern
- Understand the trade-offs between different communication patterns
- Design effective communication architectures using the right patterns
- Troubleshoot common issues related to communication pattern choice

## Communication Pattern Comparison

### Key Differences Table

| Feature | Topic | Service | Action |
|---------|-------|---------|--------|
| **Pattern** | Publish/Subscribe | Request/Reply | Goal/Feedback/Result |
| **Synchronicity** | Asynchronous | Synchronous | Synchronous (with async feedback) |
| **Response** | No direct response | Guaranteed response | Result upon completion |
| **Duration** | Continuous | Short-lived | Potentially long-lived |
| **Feedback** | No progress updates | No progress updates | Continuous feedback |
| **Cancellation** | No cancellation | No cancellation | Cancelable |
| **Reliability** | Best-effort or reliable | Reliable | Reliable |
| **Use Case** | Streaming data | Configuration/queries | Long-running tasks |

## Detailed Comparison

### 1. Topic Pattern

**Characteristics:**
- Asynchronous communication
- Multiple publishers/subscribers
- Data streams continuously or periodically
- No guarantee of message receipt
- Fire-and-forget model

**Example Implementation:**
```python
# Publisher
from std_msgs.msg import Float32
publisher = node.create_publisher(Float32, 'temperature', 10)

# Subscriber
def temp_callback(msg):
    print(f"Temperature: {msg.data}")
subscription = node.create_subscription(Float32, 'temperature', temp_callback, 10)
```

**When to Use Topics:**
- Sensor data streams (camera, LIDAR, IMU)
- Robot state publishing (joint states, odometry)
- Continuous status updates
- Broadcasting information to multiple nodes
- Real-time data where missed messages are acceptable

### 2. Service Pattern

**Characteristics:**
- Synchronous request/reply communication
- One client, one server
- Guaranteed response
- Short-lived operations
- Blocking call semantics

**Example Implementation:**
```python
# Server
from example_interfaces.srv import AddTwoInts
def add_callback(request, response):
    response.sum = request.a + request.b
    return response
service = node.create_service(AddTwoInts, 'add_two_ints', add_callback)

# Client
from example_interfaces.srv import AddTwoInts
client = node.create_client(AddTwoInts, 'add_two_ints')
request = AddTwoInts.Request()
request.a = 2
request.b = 3
future = client.call_async(request)
```

**When to Use Services:**
- Configuration changes
- Data queries that require immediate response
- Validation requests
- One-time processing tasks
- Calibration requests

### 3. Action Pattern

**Characteristics:**
- Goal-oriented communication
- Long-running operations
- Feedback during execution
- Cancelable operations
- Success/failure results

**Example Implementation:**
```python
# Server
from example_interfaces.action import Fibonacci
def execute_fibonacci(goal_handle):
    feedback_msg = Fibonacci.Feedback()
    result = Fibonacci.Result()
    # Implementation with feedback and cancellation checks
    goal_handle.succeed()
    return result
action_server = ActionServer(node, Fibonacci, 'fibonacci', execute_callback=execute_fibonacci)

# Client
from example_interfaces.action import Fibonacci
action_client = ActionClient(node, Fibonacci, 'fibonacci')
goal_msg = Fibonacci.Goal()
goal_msg.order = 10
future = action_client.send_goal_async(goal_msg)
```

**When to Use Actions:**
- Navigation to a goal
- Long-running processing tasks
- Tasks that provide progress feedback
- Operations that might need to be canceled
- Complex robot maneuvers

## Choosing the Right Pattern: Decision Matrix

### Decision Tree for Communication Pattern Selection

```
START: Need to communicate?
│
├─ Need immediate response? ──┐
│                              │
├─ YES ──┐                    │
│         │                    │
│         ├─ Short operation? ─┤
│         │                   │
│         ├─ YES ─ Service    │
│         │                   │
│         └─ NO ──┐           │
│                  │           │
│                  ├─ Cancelable? ──┐
│                  │                │
│                  ├─ YES ─ Action │
│                  │                │
│                  └─ NO ─ Service │
│                                  │
└─ NO ──┐                         │
         │                         │
         ├─ Continuous stream? ────┤
         │                        │
         ├─ YES ─ Topic          │
         │                        │
         └─ NO ──┐               │
                  │               │
                  ├─ Long operation? ──┐
                  │                    │
                  ├─ YES ──┐          │
                  │         │          │
                  │         ├─ Cancelable? ──┐
                  │         │                │
                  │         ├─ YES ─ Action │
                  │         │                │
                  │         └─ NO ─ Service │
                  │                          │
                  └─ NO ──┐                 │
                           │                 │
                           └─ Short ─ Topic │
                                         │
                                         └─ Need feedback? ──┐
                                                                 │
                                                                 ├─ YES ─ Action
                                                                 │
                                                                 └─ NO ─ Service
```

## Practical Examples and Use Cases

### Sensor Data Management

```python
# Topics for continuous sensor data
class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        
        # Multiple sensors publishing different data streams
        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Timer to publish sensor data periodically
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

# This is appropriate because:
# - Sensor data is continuous
# - Multiple subscribers might need the data
# - Slight delays are acceptable
# - Some data loss might be acceptable for real-time performance
```

### Robot Configuration System

```python
# Service for robot configuration
class RobotConfigService(Node):
    def __init__(self):
        super().__init__('robot_config_service')
        self.config_service = self.create_service(
            SetParameters, 'set_robot_config', self.set_config_callback)
    
    def set_config_callback(self, request, response):
        # Validate and apply configuration
        try:
            # Apply new configuration
            self.apply_configuration(request.parameters)
            response.success = True
            response.message = "Configuration applied successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed to apply configuration: {str(e)}"
        
        return response

# This is appropriate because:
# - Configuration requires immediate validation
# - Client needs confirmation of success/failure
# - Operation is relatively short
# - Must be reliable (service QoS)
```

### Navigation System

```python
# Action for navigation
class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self.nav_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_navigation,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
    
    def execute_navigation(self, goal_handle):
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()
        
        # Navigate with feedback and cancellation capability
        for step in range(100):  # Simplified navigation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.result_code = -1  # CANCELED
                return result
            
            # Update navigation progress
            feedback_msg.current_pose = self.get_current_pose()
            feedback_msg.distance_remaining = self.calculate_distance_remaining()
            
            goal_handle.publish_feedback(feedback_msg)
            
            # Navigate to goal (simplified)
            time.sleep(0.1)
        
        goal_handle.succeed()
        result.result_code = 1  # SUCCESS
        return result

# This is appropriate because:
# - Navigation is a long-running task
# - Progress feedback is valuable
# - Operation might need cancellation
# - Result is important after completion
```

## Performance Considerations

### Topics Performance

**Advantages:**
- High throughput for data streaming
- Low latency for real-time data
- Multiple subscribers without server load increase
- Decoupled sender/receiver timing

**Disadvantages:**
- No acknowledgment of delivery
- Potential for high network usage
- Memory usage for message queueing

```python
# Optimized topic publishing
class OptimizedSensorPublisher(Node):
    def __init__(self):
        super().__init__('optimized_sensor_publisher')
        
        # Use appropriate QoS for different data types
        self.hazardous_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.debug_qos = QoSProfile(
            depth=1,  # Limit buffer to save memory
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.critical_pub = self.create_publisher(CriticalData, 'critical', self.hazardous_qos)
        self.debug_pub = self.create_publisher(DebugData, 'debug', self.debug_qos)
```

### Services Performance

**Advantages:**
- Guaranteed request/reply delivery
- Synchronous processing guarantees
- Simple request/response logic

**Disadvantages:**
- Blocking client execution
- Limited to one-to-one communication
- Potential for server bottlenecks

```python
# Optimized service with timeout handling
class OptimizedServiceClient(Node):
    def __init__(self):
        super().__init__('optimized_service_client')
        self.client = self.create_client(ParameterService, 'set_parameter')
    
    def call_service_with_timeout(self, req, timeout=5.0):
        # Check if service is available
        if not self.client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('Service not available')
            return None
        
        # Make asynchronous call to avoid blocking
        future = self.client.call_async(req)
        
        # Wait for result with timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.done():
            return future.result()
        else:
            self.get_logger().error('Service call timed out')
            return None
```

### Actions Performance

**Advantages:**
- Progress feedback during execution
- Cancellation capability
- Suitable for long-running tasks
- Result delivery guaranteed

**Disadvantages:**
- More complex implementation
- Higher resource usage
- More network overhead

```python
# Optimized action with adaptive feedback
class OptimizedActionServer(Node):
    def __init__(self):
        super().__init__('optimized_action_server')
        self.action_server = ActionServer(
            self,
            LongRunningTask,
            'long_task',
            execute_callback=self.execute_long_task,
            goal_callback=self.goal_callback)
    
    def execute_long_task(self, goal_handle):
        start_time = self.get_clock().now()
        feedback_msg = LongRunningTask.Feedback()
        result = LongRunningTask.Result()
        
        # Calculate appropriate feedback interval
        total_steps = goal_handle.request.steps
        feedback_interval = max(1, total_steps // 50)  # Max 50 feedback messages
        
        for i in range(total_steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.completion_percentage = (i / total_steps) * 100.0
                return result
            
            # Perform actual work
            self.perform_work_step(i)
            
            # Send feedback at calculated interval
            if i % feedback_interval == 0:
                feedback_msg.progress = (i / total_steps) * 100.0
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                feedback_msg.estimated_time_remaining = \
                    (elapsed / (i + 1)) * (total_steps - i - 1)
                
                goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        result.completion_percentage = 100.0
        return result
```

## Common Mistakes and Anti-patterns

### 1. Topic Anti-patterns

```python
# BAD: Using topics for configuration
publisher.publish(ConfigurationRequest())  # No guarantee of response

# GOOD: Use services for configuration
future = config_client.call_async(config_request)

# BAD: Publishing large messages too frequently
# Large point clouds at 30Hz on a topic

# GOOD: Use appropriate frequency and message size
# Or use compression/decimation for large data
```

### 2. Service Anti-patterns

```python
# BAD: Using services for sensor data streaming
# This would block the server for each request

# GOOD: Use topics for continuous data
sensor_publisher.publish(sensor_data)

# BAD: Using services for long-running tasks
# Client blocks potentially for minutes

# GOOD: Use actions for long tasks
navigation_goal = NavigateToPose.Goal()
action_client.send_goal(navigation_goal)
```

### 3. Action Anti-patterns

```python
# BAD: Using actions for simple queries
# Overly complex for simple operations

# GOOD: Use services for simple queries
response = simple_query_service.call_async(query_request)

# BAD: Sending goals that should be continuous state
# Like current robot position (use topics instead)

# GOOD: Use actions for goal-oriented tasks
action_client.send_goal(navigate_goal)
```

## Migration Between Patterns

### When to Change Patterns

```python
# Example 1: Topic to Service (when you need guarantees)
# Before: Publishing configuration
config_publisher.publish(new_config)

# After: Using service for guaranteed delivery
future = config_service.call_async(config_request)

# Example 2: Service to Action (when operation becomes long-running)
# Before: Service that blocks for long time
response = navigation_service.call_async(nav_request)  # Blocks for minutes

# After: Using action for progress feedback
action_client.send_goal(nav_action_goal)  # Non-blocking, with feedback
```

## Best Practices Summary

### Pattern Selection Guidelines

1. **Use Topics When:**
   - Broadcasting information to multiple subscribers
   - Streaming continuous data (sensors, states)
   - Real-time performance is critical
   - Some message loss is acceptable
   - No response is needed

2. **Use Services When:**
   - Need guaranteed request/reply communication
   - Operation is relatively short (< 1s typical)
   - Client must wait for result
   - Performing configuration changes
   - Making simple queries

3. **Use Actions When:**
   - Operation is long-running (> 1s typical)
   - Progress feedback is valuable
   - Operation might need cancellation
   - Result is important but client doesn't block
   - Task has intermediate states

### Architecture Considerations

```python
# Example: Complete system using all patterns appropriately
class RoboticSystem(Node):
    def __init__(self):
        super().__init__('robotic_system')
        
        # Topics: Continuous sensor/state information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.status_pub = self.create_publisher(Status, 'robot_status', 10)
        
        # Services: Configuration and immediate queries
        self.config_srv = self.create_service(SetConfig, 'set_config', self.config_callback)
        self.query_srv = self.create_service(GetState, 'get_state', self.state_callback)
        
        # Actions: Long-running tasks
        self.nav_server = ActionServer(self, NavigateToPose, 'navigate', self.nav_execute)
        self.arm_server = ActionServer(self, MoveArm, 'move_arm', self.arm_execute)
```

## Summary

This submodule provided a comprehensive comparison of the three main communication patterns in ROS 2:

- **Topics**: Best for continuous data streaming and broadcasting
- **Services**: Best for request/reply operations requiring immediate responses
- **Actions**: Best for long-running, goal-oriented tasks with feedback

The choice between patterns depends on specific requirements like response time, data continuity, feedback needs, and cancellation capabilities. The next submodule will provide practical exercises to apply these concepts in real-world scenarios.