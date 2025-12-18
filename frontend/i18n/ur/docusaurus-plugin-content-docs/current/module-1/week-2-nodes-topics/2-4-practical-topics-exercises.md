---


sidebar_position: 4
difficulty: beginner


---
# 2.4: امتی ز - ِsoswaunssڈ ٹ آ آ پک پک پک پک یs یs یs s نs j نsas jnowaumamamama

## ج a ج

یہ ایلل ایل ایل ایل ایل بل بل ڈ بل ڈ بل ڈ ایس اے ایس ماؤس ماکلی یک ایس ک میکل یک ایس ریو بوبو سوسولین سوسوسوم کے ساساسات آئیا یک سعودہ ہ بوؤاؤاؤاؤؤس buaauauauauauauauauauauauauauauauauauauauauauauauauauauauauauauauauauauauauauaauauauaus ssad

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- ڈیز aaun arss ک v jn ف a ذ ک r یں
- ک saum mauss ج کی ق ssm یں bnaa ئیں کے l یے maucoi ص iaul ی
- Qos کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی ط ط ط ط ط ط ط ط L یے L یے Mautli ڈیٹ ڈیٹ کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی
- میبو غ غ l طی ک l طی snbaualn ے کے ll یے ٹ a پک mwaaulat ک o jna فذ ina فذ
۔

## vr ک JAS 1: Muliہ noobo rroobobo ssیsm

###
یسس روبوبو سیسسسم کے سیسیستس سنسسیسر ، اِناللرولر ، اوسوسروس مانیور نوزو وکسوس کے

### عمات

1. ** تالیق ک r یں
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_system
```
2. **Create ایک custom message** کے لیے روبوٹ state (`msg/RobotState.msg`):
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
3. **Create کا/کی sensor نود** (`robot_system/robot_system/sensor_node.py`):
```python
import rclpy
سے rclpy.نود import نود
سے std_msgs.msg import Float32
سے geometry_msgs.msg import Twist
سے sensor_msgs.msg import LaserScan
import random

class SensorNode:
    def __init__(self):
        super().__init__('sensor_node')
        
        # Create publishers کے لیے different sensor data
        self.laser_pub = self.create_publisher(
            LaserScan, 'laser_scan', 10)
        self.odom_pub = self.create_publisher(
            Float32, 'odometry', 10)
        
        # Create timer کے لیے simulated sensor readings
        self.timer = self.create_timer(0.1, self.publish_sensors)  # 10 Hz
        self.get_logger().info

    def publish_sensors(self):
        # Publish simulated laser scan
        laser_msg = LaserScan()
        laser_msg.header.stamp = self.get_clock().اب().to_msg()
        laser_msg.header.frame_id = 'laser_frame'
        laser_msg.angle_min = -1.57  # -90 degrees
        laser_msg.angle_max = 1.57   # 90 degrees
        laser_msg.angle_increment = 0.1
        laser_msg.range_min = 0.1
        laser_msg.range_max = 10.0
        laser_msg.ranges = [random.uniform(1.0, 5.0) کے لیے _ میں range(32)]  # 32 range values
        
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
4. **Create کا/کی controller نود** (`robot_system/robot_system/controller_node.py`):
```python
import rclpy
سے rclpy.نود import نود
سے geometry_msgs.msg import Twist
سے sensor_msgs.msg import LaserScan
import math

class ControllerNode:
    def __init__(self):
        super().__init__('controller_node')
        
        # Create پبلشر کے لیے velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create سبسکرائیبر کے لیے laser data
        self.laser_sub = self.create_subscription(
            LaserScan, 'laser_scan', self.laser_callback, 10)
        
        # Control variables
        self.safe_distance = 1.0  # meters
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 0.3  # rad/s
        
        self.get_logger().info

    def laser_callback(self, msg):
        # Find کا/کی closest obstacle
        min_distance = min(msg.ranges)
        
        # Create twist message
        twist = Twist()
        
        اگر min_distance < self.safe_distance:
            # Obstacle detected, turn away
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            self.get_logger().info(f'Obstacle detected! Distance: {min_distance:.2f}m, turning')
        else:
            # Path clear, move forward
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.get_logger().info(f'Moving forward, min distance: {min_distance:.2f}m')
        
        # Publish کمانڈ
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()
```
5. **Create کا/کی monitor نود** (`robot_system/robot_system/monitor_node.py`):
```python
import rclpy
سے rclpy.نود import نود
سے geometry_msgs.msg import Twist
سے std_msgs.msg import Float32
import time

class MonitorNode:
    def __init__(self):
        super().__init__('monitor_node')
        
        # Subscribe کو کمانڈ velocities اور odometry
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10)
        self.odom_sub = self.create_subscription(
            Float32, 'odometry', self.odom_callback, 10)
        
        # Track performance metrics
        self.cmd_count = 0
        self.odom_count = 0
        self.last_cmd_time = self.get_clock().اب()
        self.last_odom_time = self.get_clock().اب()
        
        self.get_logger().info

    def cmd_callback(self, msg):
        current_time = self.get_clock().اب()
        time_diff = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        self.cmd_count += 1
        self.last_cmd_time = current_time
        
        # Log کمانڈ info
        self.get_logger().info(
            f'CMD #{self.cmd_count}: Vx={msg.linear.x:.2f}, Wz={msg.angular.z:.2f}, '
            f'Hz={1.0/time_diff اگر time_diff>0 else 0:.2f}')

    def odom_callback(self, msg):
        current_time = self.get_clock().اب()
        time_diff = (current_time - self.last_odom_time).nanoseconds / 1e9
        
        self.odom_count += 1
        self.last_odom_time = current_time
        
        # Log odometry info
        self.get_logger().info(
            f'ODOM #{self.odom_count}: Pos={msg.data:.2f}, '
            f'Hz={1.0/time_diff اگر time_diff>0 else 0:.2f}')

def main(args=None):
    rclpy.init(args=args)
    monitor_node = MonitorNode()
    rclpy.spin(monitor_node)
    monitor_node.destroy_node()
    rclpy.shutdown()
```
## vr ک iass 2: ک Saum پیغ amat amat amat amat amas Qos ک s ک v jna فذ ک rna

###
اوور بونا ئیں ک saum پیغ amat astaamal ک کے SAAT ھ SAAT ھ Munassb Qus atratiautahbaat کے mautli ڈیٹ asasam کی

### عمات

1.
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<پیکیج format="3">
  <name>robot_system</name>
  <version>0.1.0</version>
  <description>روبوٹ سسٹم کے ساتھ multiple نوڈز</description>
  <maintainer email="user@مثال.com">User</maintainer>
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
</پیکیج>
```
2.
```python
سے setuptools import ترتیب
سے glob import glob
import os

package_name = 'robot_system'

ترتیب(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include any launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@مثال.com',
    description='روبوٹ سسٹم کے ساتھ multiple نوڈز',
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
3. **Create ایک launch file** (`robot_system/launch/robot_system_launch.py`):
```python
سے launch import LaunchDescription
سے launch_ros.ایکشنز import نود

def generate_launch_description():
    return LaunchDescription([
        نود(
            پیکیج='robot_system',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
            # Set parameters اگر needed
            # parameters=[{'param_name': 'param_value'}]
        ),
        نود(
            پیکیج='robot_system',
            executable='controller_node',
            name='controller_node',
            output='screen',
        ),
        نود(
            پیکیج='robot_system',
            executable='monitor_node',
            name='monitor_node',
            output='screen',
        ),
    ])
```
4.
```bash
cd ~/ros2_ws
colcon build --packages-select robot_system
source install/ترتیب.bash
ros2 launch robot_system robot_system_launch.py
```
## vr ک JAS 3: اروورورمینس ٹیٹیو OWR آپٹائزیشن

###
ک a/کی ک arard گی ک a ta ک ک ک ک ک ک ک ک ک ک asasasum OWR آپٹائز یہ.

### عمات

1.
```python
# Add کو controller نود
سے rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ControllerNode:
    def __init__(self):
        super().__init__('controller_node')
        
        # Use appropriate QoS کے لیے laser data
        laser_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # مزید reliable QoS کے لیے commands
        cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Create پبلشر اور سبسکرائیبر کے ساتھ appropriate QoS
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', cmd_qos)
        self.laser_sub = self.create_subscription(
            LaserScan, 'laser_scan', self.laser_callback, laser_qos)
        
        # Performance tracking
        self.processing_times = []
        self.get_logger().info

    def laser_callback(self, msg):
        start_time = self.get_clock().اب()
        
        # آپ کا existing laser processing code یہاں
        min_distance = min(msg.ranges)
        twist = Twist()
        
        اگر min_distance < self.safe_distance:
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        
        self.cmd_pub.publish(twist)
        
        # Track processing time
        end_time = self.get_clock().اب()
        processing_time = (end_time - start_time).nanoseconds / 1e6  # milliseconds
        self.processing_times.append(processing_time)
        
        اگر len(self.processing_times) > 100:
            self.processing_times = self.processing_times[-100:]
        
        avg_time = sum(self.processing_times) / len(self.processing_times)
        self.get_logger().info(f'Processing time: {processing_time:.2f}ms, Avg: {avg_time:.2f}ms')
```
2.
```bash
# Monitor ٹاپکس
ros2 ٹاپک hz /laser_scan
ros2 ٹاپک hz /cmd_vel

# Monitor نوڈز
ros2 run اوپر اوپر

# Monitor کے ساتھ کمانڈ-line tools
ros2 ٹاپک bw
```
## vr ک ک s 4: اورس کی baaababi ss ے nmaun ے mu یں iSrabi

###
mabbwau غ l طی ک v snnbauln ے کے باری ے مولوکو

### عمات

1.
```python
import rclpy
سے rclpy.نود import نود
سے sensor_msgs.msg import LaserScan
import random
سے rclpy.qos import QoSProfile

class RobustSensorNode:
    def __init__(self):
        super().__init__('robust_sensor_node')
        
        # Create publishers کے ساتھ different QoS کے لیے different data
        reliable_qos = QoSProfile(depth=10, reliability=1)  # RELIABLE
        best_effort_qos = QoSProfile(depth=5, reliability=0)  # BEST_EFFORT
        
        self.laser_pub = self.create_publisher(
            LaserScan, 'laser_scan', reliable_qos)
        
        # Timer کے ساتھ error handling
        self.timer = self.create_timer(0.1, self.publish_sensors)
        self.error_count = 0
        self.get_logger().info

    def publish_sensors(self):
        try:
            # Simulate occasional sensor errors
            اگر random.random() < 0.05:  # 5% error rate
                self.get_logger().warn('Simulated sensor error')
                raise Exception("Sensor reading error")
            
            # Create اور publish laser scan message
            laser_msg = LaserScan()
            laser_msg.header.stamp = self.get_clock().اب().to_msg()
            laser_msg.header.frame_id = 'laser_frame'
            laser_msg.angle_min = -1.57
            laser_msg.angle_max = 1.57
            laser_msg.angle_increment = 0.1
            laser_msg.range_min = 0.1
            laser_msg.range_max = 10.0
            laser_msg.ranges = [random.uniform(1.0, 5.0) کے لیے _ میں range(32)]
            
            self.laser_pub.publish(laser_msg)
            اگر self.error_count > 0:
                self.error_count = 0  # Reset error counter پر success
                self.get_logger().info
                
        except Exception کے طور پر e:
            self.error_count += 1
            self.get_logger().error(f'Sensor error #{self.error_count}: {e}')
            
            # Implement recovery strategy بعد N consecutive errors
            اگر self.error_count >= 5:
                self.get_logger().error('Too many consecutive errors, initiating recovery...')
                self.attempt_recovery()

    def attempt_recovery(self):
        # Simulate recovery attempt
        self.get_logger().info('Attempting sensor recovery...')
        # میں ایک real سسٹم, یہ _MAYBE_ reinitialize ہارڈ ویئر, etc.
        self.error_count = 0
        self.get_logger().info('Recovery attempt completed')

def main(args=None):
    rclpy.init(args=args)
    sensor_node = RobustSensorNode()
    
    try:
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        sensor_node.get_logger().info
    finally:
        sensor_node.destroy_node()
        rclpy.shutdown()
```
## vr ک jais 5: sasusm anaphri یش n ٹی s ٹ n گ

###
ٹی s ٹ ک a/کی maumul ssusm ossm oass nwn کے mababachn mwwaaaulat کی کی کی کی کی کی

### عمات
1. **Create ایک test script** (`robot_system/test_system.py`):
```python
import rclpy
سے rclpy.نود import نود
سے geometry_msgs.msg import Twist
سے sensor_msgs.msg import LaserScan
سے std_msgs.msg import Float32
import time

class SystemTester:
    def __init__(self):
        super().__init__('system_tester')
        
        # Subscribe کو تمام relevant ٹاپکس
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
        
        self.get_logger().info

    def cmd_callback(self, msg):
        self.cmd_count += 1

    def laser_callback(self, msg):
        self.laser_count += 1

    def odom_callback(self, msg):
        self.odom_count += 1

    def run_test(self):
        self.test_step += 1
        
        اگر self.test_step == 1:
            self.get_logger().info
            self.get_logger().info('Initial counts - Cmd: {}, Laser: {}, Odom: {}'.format(
                self.cmd_count, self.laser_count, self.odom_count))
        elif self.test_step == 2:
            current_counts = f'Cmd: {self.cmd_count}, Laser: {self.laser_count}, Odom: {self.odom_count}'
            self.get_logger().info
            
            # Verify وہ messages ہیں flowing
            اگر تمام:
                self.get_logger().info
            else:
                self.get_logger().error
        elif self.test_step == 3:
            final_counts = f'Cmd: {self.cmd_count}, Laser: {self.laser_count}, Odom: {self.odom_count}'
            self.get_logger().info(f'Final counts: {final_counts}')
            self.get_logger().info
            self.test_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    tester = SystemTester()
    rclpy.spin(tester)
    tester.destroy_node()
    rclpy.shutdown()
```
2.
```python
entry_points={
    'console_scripts': [
        'sensor_node = robot_system.sensor_node:main',
        'controller_node = robot_system.controller_node:main',
        'monitor_node = robot_system.monitor_node:main',
        'system_tester = robot_system.test_system:main',  # Add یہ line
    ],
},
```
3.
```bash
# ٹرمنل 1: Start کا/کی روبوٹ سسٹم
cd ~/ros2_ws
colcon build --packages-select robot_system
source install/ترتیب.bash
ros2 run robot_system sensor_node & ros2 run robot_system controller_node & ros2 run robot_system monitor_node

# ٹرمنل 2: Run کا/کی سسٹم tester
source install/ترتیب.bash
ros2 run robot_system system_tester
```
## خ LAA صہ

یہ یہ ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک نیچے
- نوڈ تخلیق اوار مینجمنٹ
- ٹ aa پک mwa ص lat -پیٹ rn
- ک saum پیغ am کی کی کی کی کی کی کی کی کی کی ........... پیغ remelly
- Qos t شکی l
- اورس کی baaaabی ssے snamaunے maیں خrabi
- سیسم ٹیسٹنگ اورس کی توثیق

## اعامہ النجز

1.
2
3.
4.
5.

یہ ک/کی submodules کے ll یے یے tat 2 ک a maausol 1 arn noce oraus یا aaus maus ros ros ros ros ros ros 2 mauml -acrata ہے۔
