---


sidebar_position: 4
difficulty: beginner


---
# 3.4: امتی کی کی کی کی کی کی - موولات ش شaniحmamam

## ج a ج

یہ الل l ی ی ہ ف ف کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- ایہان مامل
- ٹ a پک- پ- پ پ پ پ پ پ پ پ-
- سروس- پ پ r muna ی tacl awrss ک oaur ی کے n ظ aaam bnaa ئیں
-aa یش n پ r Mubn ی Maud پ r manbn ی ٹ asaus ک پ sr aml drididid ک ri
- ک araprd گی کی خص swaut ک a mwaaun ہ ک ri یں - mautli nmonwuch ک a
- ڈیبگ اوس نِنمونوں کے مووبی بِبِبن موباؤلات اِو بِبیبال بن

## vr ک JAS 1: ممول روبواوس ساسسم ڈیز aaid

###
یس کے

### ی SSSM ی SR

ہم اوسس روبوبی بِشن سیسسسم کے سعتا کو بنیک گے:
- **ٹاپکس**: Sensor data, روبوٹ state, سسٹم status
.
- **ایکشنز**: Navigation کو waypoints, manipulation tasks

### ایم آر اے پی ایل ہ 1: ک A/کی کی بونا ئیں
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python communication_exercises
```
### mrucl ہ 2: ک saum پیغ amat کی wauaaua ح ک r یں
Create ایک file `msg/RobotStatus.msg`:
```
# RobotStatus.msg
std_msgs/Header header
float32 battery_level
bool is_charging
string status_text
int32 error_code
```
Create ایک file `ایکشن/MoveToLocation.ایکشن`:
```
# MoveToLocation.ایکشن
string location_name
float32 target_x
float32 target_y
float32 target_theta
---
bool success
string message
int32 error_code
---
float32 current_x
float32 current_y
float32 distance_remaining
string status
```
### mriqul ہ 3: ai پ ڈیٹ پیکیج. XML
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<پیکیج format="3">
  <name>communication_exercises</name>
  <version>0.1.0</version>
  <description>مواصلات pattern exercises</description>
  <maintainer email="user@مثال.com">User</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>example_interfaces</depend>

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
### mriqul ہ 4: پ ڈیٹ ڈیٹ ک ک ک ک ک ک ڈیٹ ڈیٹ ے پی
```python
سے setuptools import ترتیب
سے glob import glob
import os

package_name = 'communication_exercises'

ترتیب(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@مثال.com',
    description='مواصلات pattern exercises',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = communication_exercises.sensor_publisher:main',
            'service_server = communication_exercises.service_server:main',
            'action_server = communication_exercises.action_server:main',
            'system_monitor = communication_exercises.system_monitor:main',
        ],
    },
)
```
## vr ک ja یک s 2: سنتسر پblیsr (ٹ aa پک s)

###
یک سیسنسر بنائیں پبلیسسسر وِسیسس ایس این ایس ایس ایس کے کے کے l یے l یے l یے l یے l یے l یے jslslsl asasauriumni گ aa ک asamalis ک samalis ک samalis samal ک samal asal asamal ک samal asamal ک samal asamal ک samal asamal ک samal asamal asamal ک

### n ف aa ذ
Create `communication_exercises/communication_exercises/sensor_publisher.py`:
```python
import rclpy
سے rclpy.نود import نود
سے sensor_msgs.msg import LaserScan, Imu
سے geometry_msgs.msg import Twist
سے std_msgs.msg import Float32
سے communication_exercises.msg import RobotStatus
import random
import math

class SensorPublisher:
    def __init__(self):
        super().__init__('sensor_publisher')
        
        # ٹاپک publishers کے لیے continuous sensor data
        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.odom_pub = self.create_publisher(Float32, 'odometry', 10)
        self.status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        
        # Timer کے لیے publishing sensor data (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_sensors)
        
        # روبوٹ state
        self.position_x = 0.0
        self.position_y = 0.0
        self.battery_level = 100.0
        
        self.get_logger().info

    def publish_sensors(self):
        # Publish laser scan
        laser_msg = LaserScan()
        laser_msg.header.stamp = self.get_clock().اب().to_msg()
        laser_msg.header.frame_id = 'laser_frame'
        laser_msg.angle_min = -math.pi / 2
        laser_msg.angle_max = math.pi / 2
        laser_msg.angle_increment = math.pi / 180  # 1 degree
        laser_msg.time_increment = 0.0
        laser_msg.scan_time = 0.1
        laser_msg.range_min = 0.1
        laser_msg.range_max = 10.0
        
        # Generate simulated laser ranges
        num_ranges = int((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment) + 1
        laser_msg.ranges = [random.uniform(0.5, 5.0) کے لیے _ میں range(num_ranges)]
        
        self.laser_pub.publish(laser_msg)
        
        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().اب().to_msg()
        imu_msg.header.frame_id = 'imu_frame'
        
        # Simulate slight movement
        imu_msg.linear_acceleration.x = random.uniform(-0.1, 0.1)
        imu_msg.linear_acceleration.y = random.uniform(-0.1, 0.1)
        imu_msg.linear_acceleration.z = 9.8 + random.uniform(-0.1, 0.1)
        
        self.imu_pub.publish(imu_msg)
        
        # Publish odometry
        odom_msg = Float32()
        self.position_x += random.uniform(-0.05, 0.05)
        self.position_y += random.uniform(-0.05, 0.05)
        odom_msg.data = math.sqrt(self.position_x**2 + self.position_y**2)
        
        self.odom_pub.publish(odom_msg)
        
        # Publish روبوٹ status
        status_msg = RobotStatus()
        status_msg.header.stamp = self.get_clock().اب().to_msg()
        
        # Simulate battery drain
        self.battery_level -= 0.01
        اگر self.battery_level < 0:
            self.battery_level = 100.0  # Cycle کے لیے demo
            
        status_msg.battery_level = self.battery_level
        status_msg.is_charging = self.battery_level > 95.0
        status_msg.status_text = "Operational" اگر self.battery_level > 20 else "کم Battery"
        status_msg.error_code = 0
        
        self.status_pub.publish(status_msg)
        
        self.get_logger().debug(f'Sensors published - Battery: {self.battery_level:.2f}%')

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    
    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        sensor_publisher.get_logger().info
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## vr ک JAS 3: سروس سرور (سروسس)

###
یsssssssssrws srwr bunیئیں کے کے ِ

### n ف aa ذ
Create `communication_exercises/communication_exercises/service_server.py`:
```python
import rclpy
سے rclpy.نود import نود
سے std_srvs.srv import SetBool, Trigger
سے communication_exercises.srv import SetConfiguration  # آپ'll need کو define یہ
سے geometry_msgs.msg import Twist

class ServiceServer:
    def __init__(self):
        super().__init__('service_server')
        
        # سروس servers
        self.emergency_stop_srv = self.create_service(
            SetBool, 'emergency_stop', self.emergency_stop_callback)
        self.calibrate_srv = self.create_service(
            Trigger, 'calibrate_sensors', self.calibrate_callback)
        self.config_srv = self.create_service(
            SetConfiguration, 'set_configuration', self.config_callback)
        
        # پبلشر کے لیے velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # روبوٹ تشکیل parameters
        self.max_velocity = 1.0
        self.safety_distance = 1.0
        
        self.get_logger().info

    def emergency_stop_callback(self, request, response):
        self.get_logger().info(f'Emergency stop requested: {request.data}')
        
        اگر request.data:  # Stop requested
            # Publish zero velocity کو stop کا/کی روبوٹ
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_pub.publish(stop_msg)
            response.success = True
            response.message = "Emergency stop activated"
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
        else:  # Resume requested
            response.success = True
            response.message = "روبوٹ کر سکتا ہے resume operation"
            self.get_logger().info('Emergency stop deactivated')
        
        return response

    def calibrate_callback(self, request, response):
        self.get_logger().info('Starting sensor calibration...')
        
        # Simulate calibration process
        try:
            # میں ایک real سسٹم, یہ _MAYBE_ involve:
            # - Taking sensor readings میں known conditions
            # - Adjusting internal parameters
            # - Validating sensor operation
            
            # Simulate calibration process
            import time
            time.sleep(2)  # Simulate 2 seconds کا calibration
            
            response.success = True
            response.message = "Calibration completed successfully"
            self.get_logger().info('Calibration completed')
            
        except Exception کے طور پر e:
            response.success = False
            response.message = f"Calibration failed: {str(e)}"
            self.get_logger().error(f'Calibration error: {e}')
        
        return response

    def config_callback(self, request, response):
        self.get_logger().info
        
        # Process تشکیل changes
        try:
            اگر request.config_name == "max_velocity":
                new_value = float(request.config_value)
                اگر 0.0 < new_value <= 5.0:  # Validate range
                    self.max_velocity = new_value
                    response.success = True
                    response.message = f"Max velocity set کو {new_value}"
                else:
                    response.success = False
                    response.message = f"Invalid max velocity value: {new_value}. ضرور ہونا 0.0 < value <= 5.0"
                    
            elif request.config_name == "safety_distance":
                new_value = float(request.config_value)
                اگر 0.1 <= new_value <= 5.0:  # Validate range
                    self.safety_distance = new_value
                    response.success = True
                    response.message = f"Safety distance set کو {new_value}"
                else:
                    response.success = False
                    response.message = f"Invalid safety distance value: {new_value}. ضرور ہونا 0.1 <= value <= 5.0"
            else:
                response.success = False
                response.message = f"Unknown تشکیل پیرامیٹر: {request.config_name}"
        
        except ValueError:
            response.success = False
            response.message = f"Invalid value type کے لیے {request.config_name}: {request.config_value}"
        except Exception کے طور پر e:
            response.success = False
            response.message = f"تشکیل error: {str(e)}"
            self.get_logger().error(f'Config error: {e}')
        
        اگر response.success:
            self.get_logger().info
        else:
            self.get_logger().warn
        
        return response

def main(args=None):
    rclpy.init(args=args)
    service_server = ServiceServer()
    
    try:
        rclpy.spin(service_server)
    except KeyboardInterrupt:
        service_server.get_logger().info
    finally:
        service_server.destroy_node()
        rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## vr ک JAS 4: آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک آک

###
ِ ss aisn srwr buna ئیں کے llull lulmbi- چ lan ے ے ے ni ی sowauchn ٹ sas ک۔

### n ف aa ذ
Create `communication_exercises/communication_exercises/action_server.py`:
```python
import time
import math
import rclpy
سے rclpy.ایکشن import ActionServer, CancelResponse, GoalResponse
سے rclpy.نود import نود
سے communication_exercises.ایکشن import MoveToLocation
سے geometry_msgs.msg import Twist

class ActionServerNode:
    def __init__(self):
        super().__init__('action_server')
        
        # ایکشن server
        self._action_server = ActionServer(
            self,
            MoveToLocation,
            'move_to_location',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        # پبلشر کے لیے velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # روبوٹ state سمولیشن
        self.current_x = 0.0
        self.current_y = 0.0
        self.is_moving = False
        
        self.get_logger().info

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received navigation goal: {goal_request.location_name}')
        
        # Validate goal parameters
        اگر (abs(goal_request.target_x) > 100.0 یا 
            abs(goal_request.target_y) > 100.0):
            self.get_logger().warn
            return GoalResponse.REJECT
        
        self.get_logger().info('Accepting navigation goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received navigation cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info
        
        # Initialize feedback اور result
        feedback_msg = MoveToLocation.Feedback()
        result = MoveToLocation.Result()
        
        # Store original target کو calculate distance
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        start_x = self.current_x
        start_y = self.current_y
        
        # Calculate total distance کے لیے percentage calculation
        total_distance = math.sqrt((target_x - start_x)**2 + (target_y - start_y)**2)
        
        # Navigation loop
        step = 0
        max_steps = 100  # Max steps کو prevent infinite loops
        
        جب تک step < max_steps:
            # Check کے لیے cancellation
            اگر goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Navigation canceled کے ذریعے user"
                result.error_code = 1
                
                # Stop کا/کی روبوٹ
                stop_msg = Twist()
                stop_msg.linear.x = 0.0
                stop_msg.angular.z = 0.0
                self.cmd_pub.publish(stop_msg)
                self.is_moving = False
                
                self.get_logger().info('Navigation canceled')
                return result

            # Calculate distance کو target
            distance_to_target = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
            
            # Update position gradually (simulating movement)
            اگر distance_to_target > 0.1:  # اگر نہیں بند enough کو target
                # Move towards target
                direction_x = (target_x - self.current_x) / distance_to_target
                direction_y = (target_y - self.current_y) / distance_to_target
                
                # Move 1% کا total distance per step
                step_size = total_distance * 0.01
                self.current_x += direction_x * step_size
                self.current_y += direction_y * step_size
                
                # Publish velocity کمانڈ
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.5  # Move پر 0.5 m/s
                cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(cmd_msg)
                self.is_moving = True
            else:
                # Reached target
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(cmd_msg)
                self.is_moving = False
                
                break
            
            # Update اور publish feedback
            feedback_msg.current_x = self.current_x
            feedback_msg.current_y = self.current_y
            feedback_msg.distance_remaining = distance_to_target
            feedback_msg.status = f"Moving کو {goal_handle.request.location_name}"
            
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().debug(
                f'Feedback - Pos: ({self.current_x:.2f}, {self.current_y:.2f}), '
                f'Dist: {distance_to_target:.2f}')
            
            # Sleep کو simulate real movement time
            time.sleep(0.2)
            step += 1
        
        # Check اگر we successfully reached کا/کی target
        final_distance = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
        
        اگر final_distance <= 0.1:  # Within tolerance
            goal_handle.succeed()
            result.success = True
            result.message = f"Successfully reached {goal_handle.request.location_name}"
            result.error_code = 0
            self.get_logger().info(f'Navigation successful: {result.message}')
        else:
            goal_handle.abort()
            result.success = False
            result.message = f"Failed کو reach {goal_handle.request.location_name}, stopped early"
            result.error_code = 2
            self.get_logger().error(f'Navigation failed: {result.message}')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = ActionServerNode()
    
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        action_server.get_logger().info
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## vr ک jais 5: ساسم مننا ah ن نوک

###
ایس ایس ایس او ایم منہر بینائی ئیں ، sbsasasahraiaub ک ri ک so ، ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک

### n ف aa ذ
Create `communication_exercises/communication_exercises/system_monitor.py`:
```python
import rclpy
سے rclpy.نود import نود
سے rclpy.ایکشن import ActionClient
سے std_srvs.srv import SetBool, Trigger
سے communication_exercises.msg import RobotStatus
سے sensor_msgs.msg import LaserScan
سے communication_exercises.ایکشن import MoveToLocation
import random

class SystemMonitor:
    def __init__(self):
        super().__init__('system_monitor')
        
        # ٹاپک subscriptions
        self.status_sub = self.create_subscription(
            RobotStatus, 'robot_status', self.status_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # سروس clients
        self.emergency_client = self.create_client(SetBool, 'emergency_stop')
        self.calibrate_client = self.create_client(Trigger, 'calibrate_sensors')
        
        # ایکشن client
        self.nav_client = ActionClient(self, MoveToLocation, 'move_to_location')
        
        # سسٹم state
        self.current_battery = 100.0
        self.safety_mode = False
        self.obstacle_detected = False
        
        # Timer کے لیے سسٹم monitoring (1 Hz)
        self.monitor_timer = self.create_timer(1.0, self.system_monitor_callback)
        
        # Timer کے لیے navigation tasks (5 seconds apart)
        self.nav_timer = self.create_timer(5.0, self.schedule_navigation)
        self.nav_counter = 0
        
        # Wait کے لیے سروسز
        جب تک نہیں self.emergency_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info
        
        جب تک نہیں self.calibrate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info
        
        self.get_logger().info

    def status_callback(self, msg):
        self.current_battery = msg.battery_level
        
        # Check کے لیے کم battery
        اگر self.current_battery < 20.0 اور نہیں self.safety_mode:
            self.get_logger().warn
            self.activate_safety_mode()
    
    def scan_callback(self, msg):
        # Check کے لیے nearby obstacles
        min_distance = min(msg.ranges) اگر msg.ranges else float('inf')
        
        اگر min_distance < 0.5 اور نہیں self.obstacle_detected:
            self.obstacle_detected = True
            self.get_logger().warn(f'OBSTACLE DETECTED: {min_distance:.2f}m')
            
            # Emergency stop اگر very بند
            اگر min_distance < 0.2:
                self.activate_emergency_stop()
    
    def system_monitor_callback(self):
        # Log سسٹم status
        self.get_logger().info(
            f'سسٹم Status - Battery: {self.current_battery:.2f}%, '
            f'Safety: {self.safety_mode}, Obstacles: {self.obstacle_detected}')
    
    def schedule_navigation(self):
        # Every 5 seconds, try ایک navigation task
        self.nav_counter += 1
        
        اگر نہیں self.safety_mode اور نہیں self.obstacle_detected:
            # Generate random target coordinates
            target_x = random.uniform(-5.0, 5.0)
            target_y = random.uniform(-5.0, 5.0)
            
            self.get_logger().info')
            self.send_navigation_goal(target_x, target_y, f"RandomGoal_{self.nav_counter}")
        else:
            self.get_logger().info
    
    def send_navigation_goal(self, x, y, location_name):
        # Wait کے لیے ایکشن server
        اگر نہیں self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error
            return
        
        # Create اور send goal
        goal_msg = MoveToLocation.Goal()
        goal_msg.target_x = x
        goal_msg.target_y = y
        goal_msg.location_name = location_name
        
        self.get_logger().info
        
        # Send goal asynchronously
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback)
        
        future.add_done_callback(self.navigation_response_callback)
    
    def navigation_feedback_callback(self, feedback_msg):
        self.get_logger().debug(
            f'Navigation feedback: {feedback_msg.feedback.status} - '
            f'Distance: {feedback_msg.feedback.distance_remaining:.2f}m')
    
    def navigation_response_callback(self, future):
        goal_handle = future.result()
        اگر نہیں goal_handle.accepted:
            self.get_logger().error
            return
        
        self.get_logger().info
        
        # Get result future
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result.message}')
    
    def activate_safety_mode(self):
        self.safety_mode = True
        self.get_logger().warn('Safety mode activated')
    
    def activate_emergency_stop(self):
        self.get_logger().error
        
        # Call emergency stop سروس
        request = SetBool.Request()
        request.data = True
        
        future = self.emergency_client.call_async(request)
        future.add_done_callback(self.emergency_stop_callback)
    
    def emergency_stop_callback(self, future):
        try:
            response = future.result()
            اگر response.success:
                self.get_logger().info(f'Emergency stop executed: {response.message}')
            else:
                self.get_logger().error(f'Emergency stop failed: {response.message}')
        except Exception کے طور پر e:
            self.get_logger().error
    
    def calibrate_sensors(self):
        self.get_logger().info('Calibrating sensors...')
        
        future = self.calibrate_client.call_async(Trigger.Request())
        future.add_done_callback(self.calibrate_callback)
    
    def calibrate_callback(self, future):
        try:
            response = future.result()
            اگر response.success:
                self.get_logger().info(f'Sensor calibration successful: {response.message}')
            else:
                self.get_logger().error(f'Sensor calibration failed: {response.message}')
        except Exception کے طور پر e:
            self.get_logger().error

def main(args=None):
    rclpy.init(args=args)
    system_monitor = SystemMonitor()
    
    try:
        rclpy.spin(system_monitor)
    except KeyboardInterrupt:
        system_monitor.get_logger().info
    finally:
        system_monitor.destroy_node()
        rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## vr ک jais 6: ف aaullun چ ک r یں

###
mau یک یک یک ف ف ک ک ک ک ک ک ک کی a/کی پ vr ی ssusm osusr ٹی si ٹ ک a/کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی
Create `communication_exercises/launch/integrated_system_launch.py`:
```python
سے launch import LaunchDescription
سے launch_ros.ایکشنز import نود

def generate_launch_description():
    return LaunchDescription([
        # Sensor پبلشر نود
        نود(
            پیکیج='communication_exercises',
            executable='sensor_publisher',
            name='sensor_publisher',
            output='screen',
        ),
        # سروس server نود
        نود(
            پیکیج='communication_exercises',
            executable='service_server',
            name='service_server',
            output='screen',
        ),
        # ایکشن server نود
        نود(
            پیکیج='communication_exercises',
            executable='action_server',
            name='action_server',
            output='screen',
        ),
        # سسٹم monitor نود
        نود(
            پیکیج='communication_exercises',
            executable='system_monitor',
            name='system_monitor',
            output='screen',
        ),
    ])
```
## vr ک iass 7: ک arard گی کی کی ج an an چ jur mwaaun ہ

###
ٹی s ٹ Buna ئیں - ک a mwaiun ہ ک ri یں // ک araurdi گی خص خص swaut ک ک ک ک ک ک ک ک ہ ہ ہ ہ ہ ہ ک ک ک ک ک ک ک ہ ہ ہ ہ
Create `communication_exercises/test/performance_test.py`:
```python
import rclpy
سے rclpy.نود import نود
سے rclpy.qos import QoSProfile
سے std_msgs.msg import String
سے std_srvs.srv import Trigger
سے communication_exercises.ایکشن import MoveToLocation
سے rclpy.ایکشن import ActionClient
import time

class PerformanceTester:
    def __init__(self):
        super().__init__('performance_tester')
        
        # ٹاپک پبلشر کے لیے performance testing
        self.topic_publisher = self.create_publisher(String, 'perf_test_topic', 
                                                   QoSProfile(depth=10))
        
        # سروس client کے لیے performance testing
        self.service_client = self.create_client(Trigger, 'perf_test_service')
        
        # ایکشن client کے لیے performance testing
        self.action_client = ActionClient(self, MoveToLocation, 'perf_test_action')
        
        # Test variables
        self.test_count = 0
        self.max_tests = 100
        
        # Timer کے لیے performance tests
        self.topic_timer = self.create_timer(0.1, self.test_topic_performance)
        self.service_timer = self.create_timer(1.0, self.test_service_performance)
        self.action_timer = self.create_timer(2.0, self.test_action_performance)
        
        # Initialize test counters
        self.topic_start_time = None
        self.service_start_time = None
        self.action_start_time = None
        
        self.get_logger().info('Performance tester initialized')

    def test_topic_performance(self):
        اگر self.test_count >= self.max_tests:
            return
            
        # Publish ایک message اور time یہ
        self.topic_start_time = time.time()
        
        msg = String()
        msg.data = f'perf_test_message_{self.test_count}'
        self.topic_publisher.publish(msg)
        
        self.test_count += 1

    def test_service_performance(self):
        اگر نہیں self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn
            return
            
        اگر self.test_count >= self.max_tests:
            return
            
        # Call سروس اور time یہ
        self.service_start_time = time.time()
        
        future = self.service_client.call_async(Trigger.Request())
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            end_time = time.time()
            اگر self.service_start_time:
                duration = end_time - self.service_start_time
                self.get_logger().info
                self.service_start_time = None
        except Exception کے طور پر e:
            self.get_logger().error

    def test_action_performance(self):
        اگر نہیں self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn
            return
            
        اگر self.test_count >= self.max_tests:
            return
            
        # Send ایکشن goal اور time یہ
        self.action_start_time = time.time()
        
        goal_msg = MoveToLocation.Goal()
        goal_msg.target_x = 1.0
        goal_msg.target_y = 1.0
        goal_msg.location_name = f'perf_test_{self.test_count}'
        
        # Send goal asynchronously
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.action_response_callback)

    def action_response_callback(self, future):
        try:
            goal_handle = future.result()
            اگر نہیں goal_handle.accepted:
                self.get_logger().error
                return
                
            # Wait کے لیے result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.action_result_callback)
        except Exception کے طور پر e:
            self.get_logger().error

    def action_result_callback(self, future):
        try:
            result = future.result().result
            end_time = time.time()
            اگر self.action_start_time:
                duration = end_time - self.action_start_time
                self.get_logger().info
                self.action_start_time = None
        except Exception کے طور پر e:
            self.get_logger().error

def main(args=None):
    rclpy.init(args=args)
    performance_tester = PerformanceTester()
    
    try:
        rclpy.spin(performance_tester)
    except KeyboardInterrupt:
        performance_tester.get_logger().info
    finally:
        performance_tester.destroy_node()
        rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## vr ک jais 8: چ چ/کی/کی maumml Saussm

### اعدعمت ک وفمال امل امل:

1.
```bash
cd ~/ros2_ws
colcon build --packages-select communication_exercises
source install/ترتیب.bash
```
2.
```bash
ros2 launch communication_exercises integrated_system_launch.py
```
3.
```bash
# Test سروس calls
ros2 سروس call /emergency_stop std_srvs/srv/SetBool "{data: true}"

# Test ایکشن calls
ros2 ایکشن send_goal /move_to_location communication_exercises/ایکشن/MoveToLocation "{target_x: 2.0, target_y: 2.0, location_name: 'test_location'}"

# Monitor ٹاپکس
ros2 ٹاپک echo /robot_status
```
4.
```bash
# View تمام active ٹاپکس
ros2 ٹاپک list

# Monitor message rates
ros2 ٹاپک hz /scan

# Check سروسز
ros2 سروس list

# Check ایکشنز
ros2 ایکشن list
```
## خ LAA صہ

یہ اعمت وِس وِس اِر ک ایسس ایس اِس ایس ایس ایس ایس ایس ایس ایس ایس ایس ایس ایس ایس ایس: آئسس:

1.
   - ٹ a پک s کے l یے msslselsel suncer ڈیٹ a avrss کی حیثی کی کی کی کی کی کی کی ک ک ک ک ک ک ک ک ک
   - سروسیس کے کے یے یے شکی شکی شکی شکی کے کے کے کے کے کے کے کے آف آف آف آف آف کے آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف آف کے آف آف آف آف کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے
   - a یکش n ز کے llu llmba- ilalalan ے ے ے ni ی Sowauchn ٹ as ک

2.

3

4

5.

## اعامہ النجز

1.
2
3.
4.
5.

یہ ک/کی submodules کے ll یے ہف یے ک ک a maausul 1 arsrosusis awr aaun ز ma یں ros ros ros ros ros roml arata ہے۔
