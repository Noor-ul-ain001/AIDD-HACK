---


sidebar_position: 3
difficulty: advanced


---
# 2.3: ڈیز aiun کے naumunے

## ج a ج

یہ الل l ی

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- Apply advanced design patterns کے لیے ٹاپکس
-
- مناسب غلطی کو سنبھالنے کا عمل کریں
- ڈیز a ئ n یsasal العازبل - ف ف ف ف ف ف ف ف
- اعلی درییجے کی Qos taclat ک so mauri ط r یقے ss ے ساسٹامیل Alsr یں

## ٹ ٹ a ٹ ڈیز aiaun کے jinun ے

#####

ٹ a پک s ک ک v ک v ک amaal vasriے nocuss ssasusm wauaaaی Jo muclad:
```python
# Event پبلشر
سے std_msgs.msg import String

class EventPublisher:
    def __init__(self):
        super().__init__('event_publisher')
        self.event_pub = self.create_publisher(String, 'system_events', 10)
    
    def publish_event(self, event_type, details=""):
        msg = String()
        msg.data = f"{event_type}:{details}"
        self.event_pub.publish(msg)

# Usage مثال
event_publisher = EventPublisher()
event_publisher.publish_event("SENSOR_FAILURE", "Lidar sensor disconnected")
```
### 2

متعاد اعدعد وِمار کے ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ کے ذ ذ کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے
```python
سے sensor_msgs.msg import LaserScan, Imu
سے geometry_msgs.msg import Twist
سے my_robot_msgs.msg import RobotStatus  # Custom aggregated message

class DataAggregator:
    def __init__(self):
        super().__init__('data_aggregator')
        
        # Subscriptions کو multiple data sources
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
        اگر تمام([self.last_lidar, self.last_imu, self.last_cmd]):
            msg = RobotStatus()
            msg.header.stamp = self.get_clock().اب().to_msg()
            msg.lidar_data = self.last_lidar.ranges[:10]  # Simplified
            msg.imu_data.orientation = self.last_imu.orientation
            msg.کمانڈ.linear = self.last_cmd.linear
            msg.کمانڈ.angular = self.last_cmd.angular
            
            self.status_pub.publish(msg)
```
### 3

ٹ a پک sis maus یں یں یں یں یں یں یں یں
```python
# اچھا ٹاپک naming structure
# /robot_name/سسٹم/اجزاء/data_type
# Examples:
# /turtle1/sensors/lidar/scan
# /turtle1/motors/wheel_front_left/velocity
# /turtle1/control/cmd_vel
# /arm_1/joint_states/position
# /arm_1/end_effector/pose
```
### 4۔ rausast ی ہ m isn گی ک a numun ہ

نہیں کے پ پ پ پ پ پ پ پ پ پ پ پ پ پ ک ک پک پک پک کے پک پک کے کے پک کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے
```python
سے my_robot_msgs.msg import SystemState

class StateSynchronizer:
    def __init__(self):
        super().__init__('state_synchronizer')
        
        # State پبلشر
        self.state_pub = self.create_publisher(SystemState, 'system_state', 1)
        
        # State subscription
        self.state_sub = self.create_subscription(
            SystemState, 'system_state', self.state_callback, 1)
        
        # Initialize سسٹم state
        self.system_state = SystemState()
    
    def update_and_publish_state:
        # Update specific اجزاء میں سسٹم state
        اگر اجزاء == "battery":
            self.system_state.battery_level = value
        elif اجزاء == "position":
            self.system_state.position = value
        
        # Publish entire state (synchronized)
        self.system_state.header.stamp = self.get_clock().اب().to_msg()
        self.state_pub.publish(self.system_state)
    
    def state_callback(self, msg):
        # Update local state representation
        self.system_state = msg
```
## ڈیٹ asasriqulaiauchn awrs Bachna ڈ vatut ھ کی کی ص کی کی کی کی کی کی

### mwaur پیغ am ڈیز aaid
```python
# اچھا: Compact message design
# CompactSensorData.msg
float32[3] position  # Instead کا separate x, y, z
uint8 sensor_type   # Use enums instead کا strings
float32 confidence  # Single confidence value instead کا complex structure

# Avoid: Excessive data میں messages
# Don't send entire images پر regular ٹاپکس (use image_transport)
# Don't send بڑا arrays کب صرف ایک few values ہیں needed
```
### ڈیٹ a -ک mapri یش n awnsunsmauln گ
```python
سے sensor_msgs.msg import PointCloud2
import numpy کے طور پر np

class DataCompressor:
    def __init__(self):
        super().__init__('data_compressor')
        
        self.raw_sub = self.create_subscription(
            PointCloud2, 'raw_pointcloud', self.raw_callback, 1)
        self.compressed_pub = self.create_publisher(
            PointCloud2, 'compressed_pointcloud', 1)
    
    def raw_callback(self, msg):
        # Downsample یا filter data پہلے publishing
        اگر self.should_compress():
            compressed_msg = self.downsample_pointcloud(msg)
            self.compressed_pub.publish(compressed_msg)
        else:
            self.compressed_pub.publish(msg)
    
    def downsample_pointcloud(self, cloud_msg):
        # نفاذ کو reduce point cloud density
        # یہ ہے ایک simplified مثال
        return cloud_msg  # Placeholder
```
```python
سے rclpy.qos import QoSProfile

class RateLimitedPublisher:
    def __init__(self):
        super().__init__('rate_limited_publisher')
        
        # Create timer کے لیے rate limiting
        self.timer = self.create_timer(0.1, self.publish_callback)  # 10 Hz
        self.پبلشر = self.create_publisher(String, 'rate_limited_topic', 
                                             QoSProfile(depth=1))
        
        self.counter = 0
    
    def publish_callback(self):
        msg = String()
        msg.data = f"Message {self.counter}"
        self.پبلشر.publish(msg)
        self.counter += 1
```
## اعل ی Qos Qos taclat

### حقیقی waut کی ک arard گی qos
```python
سے rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
سے rclpy.qos import LivelinessPolicy

# کے لیے real-time applications
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
### ق ABL AASATMAD MOWAVALAT QOS
```python
# کے لیے mission-critical applications
critical_qos = QoSProfile(
    depth=100,  # Larger buffer کے لیے important data
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_ALL,  # Keep تمام messages
    durability=DurabilityPolicy.TRANSIENT_LOCAL  # Persist messages
)
```
### تعصب ٹ- aa یف vr ٹ mwwaaalat Qos
```python
# کے لیے اونچا-frequency, non-critical data
best_effort_qos = QoSProfile(
    depth=5,  # چھوٹا buffer کے لیے frequent updates
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE
)
```
## غ l طی ک l طی ک v snanbauln ے ma یں غ l طی

### پ بلہر کے ساٹ ھ غ l طی کی baaaabis
```python
سے rclpy.qos import QoSProfile
سے std_msgs.msg import Header

class RobustPublisher:
    def __init__(self):
        super().__init__('robust_publisher')
        
        # Use appropriate QoS کے لیے reliability
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.پبلشر = self.create_publisher(String, 'robust_topic', qos_profile)
        
        self.fallback_mode = False
        self.recovery_counter = 0
        
        # Timer کو attempt recovery
        self.recovery_timer = self.create_timer(5.0, self.recovery_callback)
    
    def publish_with_fallback(self, msg_data):
        try:
            msg = String()
            msg.data = msg_data
            self.پبلشر.publish(msg)
        except Exception کے طور پر e:
            self.get_logger().error(f'Publish failed: {e}')
            self.fallback_mode = True
            self.handle_publish_failure(msg_data)
    
    def handle_publish_failure(self, msg_data):
        # Log error کے لیے later analysis
        self.get_logger().error
        # Implement appropriate fallback behavior
    
    def recovery_callback(self):
        اگر self.fallback_mode:
            self.recovery_counter += 1
            اگر self.recovery_counter >= 3:  # Try recovery بعد 3 attempts
                self.attempt_recovery()
    
    def attempt_recovery(self):
        self.get_logger().info
        try:
            # Reinitialize پبلشر اگر needed
            qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.پبلشر = self.create_publisher(String, 'robust_topic', qos_profile)
            self.fallback_mode = False
            self.recovery_counter = 0
            self.get_logger().info
        except Exception کے طور پر e:
            self.get_logger().error
```
### sbsa ک ra ئی br کے SAAT ھ tatatous
```python
سے std_msgs.msg import Float32

class ValidatingSubscriber:
    def __init__(self):
        super().__init__('validating_subscriber')
        
        self.subscription = self.create_subscription(
            Float32, 'sensor_value', self.sensor_callback, 10)
        
        # Store previous values کے لیے change detection
        self.previous_values = []
        self.max_change_rate = 10.0  # Max acceptable change per دوسرا
        self.last_time = self.get_clock().اب()
    
    def sensor_callback(self, msg):
        current_time = self.get_clock().اب()
        time_diff = (current_time - self.last_time).nanoseconds / 1e9  # seconds
        
        # Validate message value
        اگر نہیں self.is_value_valid(msg.data):
            self.get_logger().warn(f'Invalid sensor value received: {msg.data}')
            return
        
        # Check کے لیے excessive changes
        اگر len(self.previous_values) > 0 اور time_diff > 0:
            rate_of_change = abs(msg.data - self.previous_values[-1]) / time_diff
            اگر rate_of_change > self.max_change_rate:
                self.get_logger().warn
        
        # Store value اور update time
        self.previous_values.append(msg.data)
        self.last_time = current_time
        
        # Process valid message
        self.process_sensor_value(msg.data)
    
    def is_value_valid(self, value):
        # مثال validation: check کے لیے reasonable sensor range
        return -100.0 <= value <= 100.0
    
    def process_sensor_value(self, value):
        # Process کا/کی validated sensor value
        self.get_logger().info(f'Processing sensor value: {value}')
```
## ٹ a ٹ Manaprni گ گ گ گ گ گ ٹ ٹ

### t شخیصی شخیصی bl ش r کے l ِ l ٹ ٹ a پک s
```python
سے diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class TopicDiagnostics:
    def __init__(self):
        super().__init__('topic_diagnostics')
        
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self.status_timer = self.create_timer(1.0, self.publish_diagnostics)
        
        # Track ٹاپک statistics
        self.topic_stats = {
            'topic_name': {
                'message_count': 0,
                'last_message_time': None,
                'avg_frequency': 0.0
            }
        }
    
    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().اب().to_msg()
        
        کے لیے topic_name, stats میں self.topic_stats.items():
            status = DiagnosticStatus()
            status.name = f'ٹاپک {topic_name}'
            status.hardware_id = 'ROS2'
            
            # Determine status level based پر statistics
            اگر stats['message_count'] == 0:
                status.level = DiagnosticStatus.ERROR
                status.message = 'نہیں messages received'
            elif stats['avg_frequency'] < 0.1:  # Less than 0.1 Hz
                status.level = DiagnosticStatus.WARN
                status.message = f'کم frequency: {stats["avg_frequency"]:.2f} Hz'
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
## ک اراپراڈ گی کے تالٹ

### maumur ی ​​maunaunmini ٹ کے li ِ ss owana چ a- ف raiuchnsi ی ٹ aa پک s
```python
سے std_msgs.msg import Float32MultiArray
import weakref

class EfficientHighFreqSubscriber:
    def __init__(self):
        super().__init__('efficient_subscriber')
        
        # Use appropriate QoS کے لیے اونچا-frequency data
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Float32MultiArray, 'high_freq_data', self.data_callback, qos)
        
        # Limit stored references کو prevent memory leaks
        self.processed_messages = []
        self.max_stored_messages = 100
    
    def data_callback(self, msg):
        # Process message immediately
        processed_data = self.process_message(msg)
        
        # Store limited history کو prevent memory buildup
        self.processed_messages.append(processed_data)
        اگر len(self.processed_messages) > self.max_stored_messages:
            self.processed_messages = self.processed_messages[-50:]  # Keep last 50
        
        # Don't store references کو msg اگر نہیں needed
        del msg  # Just ایک reference, doesn't free کا/کی actual message
```
## خ LAA صہ

یہ یہ ، ، یہ یہ یہ یہ یہ یہ یہ یہ یہ یہ یہ یہ یہ ک j ک j یہ j یہ j یہ j j یہ J یہ J یہ J یہ J یہ J یہ J یہ J یہ J یہ J یہ J یہ J یہ J یہ J یہ J یہ J یہ J یہ J یہ J یہ ک J یہ ، J یہ
