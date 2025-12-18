---


sidebar_position: 4
difficulty: intermediate


---
# 2.4: سوسولن اعیمالیہ ما

## ج a ج

یہ الل l ی آپ کہ کہ کے ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک کے کے کے کے کے ک ک ک ک کے کے کے کے کے ک کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے ک ک ک ک ک ک ک کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے ک ک کے ک ک کے کے کے کے کے کے کے کے کے ک ک کے کے کے کے کے کے کے ی ی ک کے کے کے ک کے ک ک ک ک ث ث ث ث

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- مکمل سمولہن مامول سکی سکریچ بنائیں
- Implement روبوٹ navigation میں سمولیشن
- کam کے stی ھ maumnoaی ssasnssr -
- Aisn ی Mr ضی کے mubah گیز Bi ی Joli پ Li an Buna ئیں
- سوسمولان کے سعتا ھ روس 2 اِسنرول سیسسم اوسوسو مربو
- کامن سامولن کے مسا ڈی ڈیبگ

## vr ک JAS 1: maumul smwlain mama tt tt athr

###
ممول ایس ایس ایس ایم ڈبلیو لین ماول کے ساسٹا ھ rewbo ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ،

### mracl ہ 1: پیکیج SAA خ at bnaa ئیں

پہلا ، ، ​​، ک ک ک/کی ض ض ض aaauri یکٹ raui ں ک v بونا ئیں بونا کے کے کے ِ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python simulation_exercises
cd simulation_exercises
```
### mriqul ہ 2: علیہم ف aaul یں bna ئیں
Create ایک directory کے لیے ہمارا worlds اور create ایک basic world file `simulation_exercises/worlds/robotics_lab.sdf`:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="robotics_lab">
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Physics تشکیل -->
    <physics name="ode" default="0" type="ode">
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Add ایک basic lab ماحول کے ساتھ obstacles -->
    <!-- Walls -->
    <model name="wall_1">
      <static>true</static>
      <pose>0 5 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="wall_2">
      <static>true</static>
      <pose>0 -5 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="wall_3">
      <static>true</static>
      <pose>5 0 1 0 0 1.57</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="wall_4">
      <static>true</static>
      <pose>-5 0 1 0 0 1.57</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Add obstacles -->
    <model name="obstacle_1">
      <static>false</static>
      <pose>2 2 0.2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.4 0.4 0.4</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.4 0.4 0.4</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.0133</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0133</iyy>
            <iyz>0</iyz>
            <izz>0.0133</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <model name="obstacle_2">
      <static>false</static>
      <pose>-2 -2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.15</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.15</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Add ایک charging station (special area) -->
    <model name="charging_station">
      <static>true</static>
      <pose>4 -4 0.05 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 1.0 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 1.0 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.2 1</ambient>
            <diffuse>0.8 0.8 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```
### mrucl ہ 3: روبو ما a ی bnna ئیں
Create کا/کی روبوٹ یوآر ڈی ایف file `simulation_exercises/یوآر ڈی ایف/differential_drive_robot.یوآر ڈی ایف`:
```xml
<?xml version="1.0"?>
<روبوٹ name="diff_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931"/>
  <xacro:property name="base_width" value="0.3"/>
  <xacro:property name="base_length" value="0.4"/>
  <xacro:property name="base_height" value="0.15"/>
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_width" value="0.025"/>
  <xacro:property name="wheel_mass" value="0.2"/>
  <xacro:property name="base_mass" value="5.0"/>
  
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="${base_mass}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>
  
  <!-- Base footprint -->
  <link name="base_footprint">
    <visual>
      <geometry>
        <cylinder radius="0.01" length="0.001"/>
      </geometry>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${base_width/2}" length="0.1"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.0001"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  
  <!-- Joint: base_footprint کو base_link -->
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${base_height/2}" rpy="0 0 0"/>
  </joint>
  
  <!-- Macro کے لیے wheels -->
  <xacro:macro name="wheel" params="prefix reflect">
    <link name="${prefix}_wheel_link">
      <visual>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0.0 0.0 0.0 1.0"/>
        </material>
      </visual>
      
      <collision>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      
      <inertial>
        <mass value="${wheel_mass}"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>
    
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel_link"/>
      <origin xyz="${base_length/2 - wheel_width/2} ${reflect * base_width/2} -${base_height/2}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>
  
  <!-- Create wheels -->
  <xacro:wheel prefix="بائیں" reflect="1"/>
  <xacro:wheel prefix="صحیح" reflect="-1"/>
  
  <!-- Camera link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.1 0.03"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.0 0.0 1.0"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.05 0.1 0.03"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="${base_length/2 - 0.025} 0 ${base_height/2}" rpy="0 0 0"/>
  </joint>
  
  <!-- IMU link -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 ${base_height/4}" rpy="0 0 0"/>
  </joint>
  
  <!-- گیزبو plugins -->
  <گیزبو reference="base_link">
    <material>گیزبو/Blue</material>
  </گیزبو>
  
  <گیزبو reference="left_wheel_link">
    <material>گیزبو/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </گیزبو>
  
  <گیزبو reference="right_wheel_link">
    <material>گیزبو/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </گیزبو>
  
  <گیزبو reference="camera_link">
    <material>گیزبو/Red</material>
  </گیزبو>

  <!-- Differential drive controller plugin -->
  <گیزبو>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <update_rate>30</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>${base_width}</wheel_separation>
      <wheel_diameter>${2 * wheel_radius}</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
    </plugin>
  </گیزبو>

  <!-- Camera plugin -->
  <گیزبو reference="camera_link">
    <sensor name="camera" type="camera">
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>10</max_depth>
      </plugin>
    </sensor>
  </گیزبو>

  <!-- IMU plugin -->
  <گیزبو reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>50</update_rate>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <frame_name>imu_link</frame_name>
        <ٹاپک>imu/data</ٹاپک>
        <serviceName>imu_service</serviceName>
      </plugin>
    </sensor>
  </گیزبو>
</روبوٹ>
```
### mracl ہ 4: ک inaphrolr tal bnaa ئیں
Create controller تشکیل file `simulation_exercises/config/diff_drive_controller.yaml`:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    use_sim_time: true

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.3
    wheel_radius: 0.05
    publish_rate: 50.0

    # Velocity commands
    cmd_vel_timeout: 0.5
    linear.x.has_velocity_limits: true
    linear.x.has_acceleration_limits: true
    linear.x.has_jerk_limits: false
    linear.x.max_velocity: 1.0
    linear.x.min_velocity: -1.0
    linear.x.max_acceleration: 2.0
    linear.x.min_acceleration: -2.0
    linear.x.max_jerk: 0.0
    linear.x.min_jerk: 0.0

    angular.z.has_velocity_limits: true
    angular.z.has_acceleration_limits: true
    angular.z.has_jerk_limits: false
    angular.z.max_velocity: 1.0
    angular.z.min_velocity: -1.0
    angular.z.max_acceleration: 2.0
    angular.z.min_acceleration: -2.0
    angular.z.max_jerk: 0.0
    angular.z.min_jerk: 0.0
```
## vr ک JAS 2: روبوبو ک na ٹ rlol maus smulun

###
آروس 2 پیغ امات ک a ک asamal ک samal ہ Juchrobo ک na ٹ rlol ک v jna فذ ک ri
Create `simulation_exercises/simulation_exercises/robot_controller.py`:
```python
#!/usr/bin/env python3
import rclpy
سے rclpy.نود import نود
سے geometry_msgs.msg import Twist, Vector3
سے nav_msgs.msg import Odometry
سے sensor_msgs.msg import LaserScan
سے tf2_ros import TransformException
سے tf2_ros.buffer import Buffer
سے tf2_ros.transform_listener import TransformListener
سے std_msgs.msg import String
import math

class RobotController:
    def __init__(self):
        super().__init__('robot_controller')
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # TF buffer اور listener کے لیے transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # روبوٹ state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.scan_ranges = []
        
        # Control parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.3  # rad/s
        self.safe_distance = 0.5  # meters
        self.target_x = 3.0  # Target x position
        self.target_y = 2.0  # Target y position
        
        # Create timer کے لیے control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info

    def odom_callback(self, msg):
        """Update روبوٹ pose سے odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convert quaternion کو yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        """Update sensor data"""
        self.scan_ranges = msg.ranges

    def control_loop(self):
        """Main control loop"""
        # Calculate distance کو target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance_to_target = math.sqrt(dx*dx + dy*dy)
        
        # Calculate angle کو target
        angle_to_target = math.atan2(dy, dx)
        angle_diff = angle_to_target - self.current_theta
        
        # Normalize angle
        جب تک angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        جب تک angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Check اگر بند کو target
        اگر distance_to_target < 0.3:
            self.get_logger().info('Reached target position!')
            self.stop_robot()
            return
        
        # Determine اگر obstacle ہے ahead
        obstacle_ahead = self.check_obstacle_ahead()
        
        # Create twist کمانڈ
        twist = Twist()
        
        اگر obstacle_ahead:
            # اگر obstacle ahead, rotate کو avoid
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
        elif abs(angle_diff) > 0.2:  # 0.2 radians ~ 11.5 degrees
            # اگر نہیں aligned کے ساتھ target, rotate
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed اگر angle_diff > 0 else -self.angular_speed
        else:
            # Move toward target
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        
        # Publish کمانڈ
        self.cmd_vel_pub.publish(twist)
        
        # Log current status
        self.get_logger().info(
            f'Position: ({self.current_x:.2f}, {self.current_y:.2f}), '
            f'Heading: {self.current_theta:.2f}, '
            f'Distance کو target: {distance_to_target:.2f}, '
            f'Obstacle ahead: {obstacle_ahead}')

    def check_obstacle_ahead(self):
        """Check اگر وہاں ہے ایک obstacle ahead based پر laser scan"""
        اگر نہیں self.scan_ranges:
            return False
            
        # Check کا/کی سامنے 30 degrees کا کا/کی laser scan
        front_start = len(self.scan_ranges) // 2 - 15  # -15 degrees سے مرکز
        front_end = len(self.scan_ranges) // 2 + 15    # +15 degrees سے مرکز
        
        اگر front_start < 0:
            front_start = 0
        اگر front_end >= len(self.scan_ranges):
            front_end = len(self.scan_ranges) - 1
        
        # Check اگر any کا کا/کی سامنے readings ہیں within safe distance
        کے لیے میں میں range(front_start, front_end):
            اگر self.scan_ranges[میں] < self.safe_distance اور نہیں math.isnan:
                return True
        
        return False

    def stop_robot(self):
        """Stop کا/کی روبوٹ"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## vr ک iass 3: سنتسر اوانضمامام اعوروس پ rwsaussn گ

###
پ راساس ڈیٹ a ک s ک s زی زی زی زی زی ہ ہ Maunwaua ی ssauncr - bin ی Adad SSANCER فی Sonathnathnau ں ک s
Create `simulation_exercises/simulation_exercises/sensor_processor.py`:
```python
#!/usr/bin/env python3
import rclpy
سے rclpy.نود import نود
سے sensor_msgs.msg import LaserScan, Imu
سے nav_msgs.msg import Odometry
سے geometry_msgs.msg import PointStamped, Vector3
سے visualization_msgs.msg import Marker
سے tf2_ros import TransformException
سے tf2_ros.buffer import Buffer
سے tf2_ros.transform_listener import TransformListener
import math
import numpy کے طور پر np
سے collections import deque
import statistics

class SensorProcessor:
    def __init__(self):
        super().__init__('sensor_processor')
        
        # Create subscribers کے لیے various sensors
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Create publishers
        self.obstacle_pub = self.create_publisher(PointStamped, '/obstacle_position', 10)
        self.marker_pub = self.create_publisher(Marker, '/obstacle_marker', 10)
        
        # TF buffer اور listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Data storage
        self.scan_ranges = []
        self.imu_data = {'orientation': None, 'angular_velocity': None, 'linear_acceleration': None}
        self.odom_data = {'position': None, 'velocity': None}
        self.obstacle_history = deque(maxlen=10)  # Keep last 10 obstacle readings
        
        # Sensor parameters
        self.safe_distance = 0.8  # meters
        self.min_obstacle_size = 0.3  # meters کو ہونا considered ایک obstacle
        
        # Create timer کے لیے processing
        self.timer = self.create_timer(0.5, self.process_sensor_data)  # 2 Hz
        
        self.get_logger().info('Sensor Processor initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.scan_ranges = msg.ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_max = msg.angle_max
        self.scan_angle_increment = msg.angle_increment
        self.scan_range_min = msg.range_min
        self.scan_range_max = msg.range_max

    def imu_callback(self, msg):
        """Process IMU data"""
        self.imu_data = {
            'orientation': msg.orientation,
            'angular_velocity': msg.angular_velocity,
            'linear_acceleration': msg.linear_acceleration
        }

    def odom_callback(self, msg):
        """Process odometry data"""
        self.odom_data = {
            'position': msg.pose.pose.position,
            'velocity': msg.twist.twist.linear
        }

    def process_sensor_data(self):
        """Main processing function"""
        اگر نہیں self.scan_ranges:
            self.get_logger().warn
            return
        
        # Process laser scan کو detect obstacles
        obstacles = self.detect_obstacles()
        
        # Publish obstacle data اگر any found
        اگر obstacles:
            # Average کا/کی obstacle positions کو smooth readings
            avg_x = statistics.mean
            avg_y = statistics.mean
            
            # Create اور publish obstacle marker
            self.publish_obstacle_marker(avg_x, avg_y)
            
            # Store obstacle کے لیے history
            self.obstacle_history.append({'x': avg_x, 'y': avg_y, 'timestamp': self.get_clock().اب()})
            
            self.get_logger().info')
        else:
            self.get_logger().info
        
        # Process اور log IMU data
        اگر self.imu_data['linear_acceleration']:
            # Calculate approximate heading change سے IMU
            angular_z = self.imu_data['angular_velocity'].z
            self.get_logger().info(f'Angular velocity: {angular_z:.3f} rad/s')
        
        # Process اور log odometry data
        اگر self.odom_data['velocity']:
            linear_speed = math.sqrt(
                self.odom_data['velocity'].x**2 + 
                self.odom_data['velocity'].y**2
            )
            self.get_logger().info(f'Current speed: {linear_speed:.3f} m/s')

    def detect_obstacles(self):
        """Detect obstacles سے laser scan data"""
        obstacles = []
        
        اگر نہیں self.scan_ranges:
            return obstacles
        
        # Group consecutive readings وہ represent کا/کی same obstacle
        current_obstacle = []
        
        کے لیے میں, range_val میں enumerate(self.scan_ranges):
            اگر range_val < self.safe_distance اور نہیں math.isnan(range_val):
                # Calculate angle کے لیے یہ reading
                angle = self.scan_angle_min + میں * self.scan_angle_increment
                
                # Convert کو Cartesian coordinates
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                
                اگر current_obstacle:
                    # Check اگر یہ point ہے بند کو کا/کی previous ایک
                    prev_x, prev_y = current_obstacle[-1]
                    distance = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
                    
                    اگر distance < 0.2:  # Points within 20cm ہیں part کا same obstacle
                        current_obstacle.append((x, y))
                    else:
                        # یہ ہے ایک نیا obstacle, save previous ایک اگر بڑا enough
                        اگر len(current_obstacle) >= 2:  # پر least 2 points کو ہونا ایک obstacle
                            center_x = sum / len(current_obstacle)
                            center_y = sum / len(current_obstacle)
                            obstacles.append({'x': center_x, 'y': center_y})
                        
                        # Start نیا obstacle
                        current_obstacle = [(x, y)]
                else:
                    current_obstacle = [(x, y)]
            else:
                # اختتام کا obstacle, save یہ اگر بڑا enough
                اگر current_obstacle اور len(current_obstacle) >= 2:
                    center_x = sum / len(current_obstacle)
                    center_y = sum / len(current_obstacle)
                    obstacles.append({'x': center_x, 'y': center_y})
                
                current_obstacle = []
        
        # Don't forget کا/کی last obstacle اگر we تھے میں ایک
        اگر current_obstacle اور len(current_obstacle) >= 2:
            center_x = sum / len(current_obstacle)
            center_y = sum / len(current_obstacle)
            obstacles.append({'x': center_x, 'y': center_y})
        
        return obstacles

    def publish_obstacle_marker(self, x, y):
        """Publish visualization marker کے لیے detected obstacle"""
        marker = Marker()
        marker.header.frame_id = 'odom'  # Use odom frame کے لیے global visualization
        marker.header.stamp = self.get_clock().اب().to_msg()
        marker.ns = 'obstacle_detection'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.ایکشن = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1  # Slightly above ground
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.2  # 20cm diameter sphere
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        marker.color.ایک = 0.8  # Alpha
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    processor = SensorProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info
    finally:
        processor.destroy_node()
        rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## vr ک iass 4: ف a ئ l an ض mamamaumlan چ ک r یں

###
لانچ ف aaul یں bnaa ئیں - شan/ک maumml susumulain maawaowl کے saesataatatttatttatttatttam anono
Create `simulation_exercises/launch/simulation_exercises.launch.py`:
```python
سے launch import LaunchDescription
سے launch.ایکشنز import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
سے launch.launch_description_sources import PythonLaunchDescriptionSource
سے launch.substitutions import LaunchConfiguration, PathJoinSubstitution
سے launch_ros.ایکشنز import نود
سے launch_ros.substitutions import FindPackageShare
سے launch.event_handlers import OnProcessExit
سے ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get کا/کی پیکیج directory
    pkg_share = get_package_share_directory('simulation_exercises')
    
    # World file path
    world_file = os.path.join(pkg_share, 'worlds', 'robotics_lab.sdf')
    
    # یوآر ڈی ایف file path
    urdf_file = os.path.join
    
    # Controller config file path
    controller_config_file = os.path.join(pkg_share, 'config', 'diff_drive_controller.yaml')
    
    # Launch گیزبو کے ساتھ کا/کی world
    گیزبو = ExecuteProcess(
        cmd=['گیزبو', '--verbose', world_file, 
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # روبوٹ State پبلشر
    robot_state_publisher = نود(
        پیکیج='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': کھلا(urdf_file).read()}
        ]
    )
    
    # Spawn entity میں گیزبو
    spawn_entity = نود(
        پیکیج='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-ٹاپک', 'robot_description',
            '-entity', 'diff_drive_robot',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )
    
    # روبوٹ controller
    robot_controller = نود(
        پیکیج='simulation_exercises',
        executable='robot_controller',
        name='robot_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Sensor processor
    sensor_processor = نود(
        پیکیج='simulation_exercises',
        executable='sensor_processor',
        name='sensor_processor',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Joint state پبلشر
    joint_state_publisher = نود(
        پیکیج='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Controller manager
    controller_manager = نود(
        پیکیج='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config_file, {'use_sim_time': True}],
        output='screen'
    )
    
    # Load اور start controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'diff_drive_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        گیزبو,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        controller_manager,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        robot_controller,
        sensor_processor,
    ])
```
## vr ک iass 5: ک Saum گیز Bubo پ la گ ann

###
maausoula ی at ی ح alat ی ی گیز کے کے کے کے کے کے کے کے کے ص ص ص ص ص ط ط ط ط ط ط ق ق ق ق ق ق ق d ک r یں۔
Create `simulation_exercises/gazebo_plugins/custom_force_plugin.cpp`:
```cpp
#include <گیزبو/گیزبو.hh>
#include <گیزبو/physics/physics.hh>
#include <گیزبو/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace گیزبو
{
  class CustomForcePlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store کا/کی model pointer کے لیے later usage
      یہ->model = _model;
      
      // Get کا/کی پہلا link (chassis) - assuming یہ's کا/کی پہلا link
      یہ->link = _model->GetLink();
      
      // Listen کو کا/کی update event. یہ event ہے broadcast every
      // سمولیشن iteration.
      یہ->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind);
          
      std::cout << "CustomForcePlugin loaded کے لیے model: " << _model->GetName() << std::endl;
    }

    // Called کے ذریعے کا/کی world update start event
    public: void OnUpdate()
    {
      // Apply ایک چھوٹا random force کو simulate environmental disturbance
      // یہ makes کا/کی سمولیشن مزید challenging اور realistic
      
      // Get current time کو apply time-varying forces
      double seconds = یہ->model->GetWorld()->SimTime().Double();
      
      // Apply ایک periodic force میں کا/کی x-direction
      double force_x = 0.1 * sin(seconds);
      double force_y = 0.05 * cos(seconds * 1.5);
      
      // Apply کا/کی force کو کا/کی link's مرکز کا mass
      ignition::math::Vector3d force(force_x, force_y, 0);
      یہ->link->AddForce(force);
    }

    // Pointer کو کا/کی model
    private: physics::ModelPtr model;
    
    // Pointer کو کا/کی link
    private: physics::LinkPtr link;
    
    // Event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register یہ plugin کے ساتھ کا/کی simulator
  GZ_REGISTER_MODEL_PLUGIN(CustomForcePlugin)
}
```
## vr ک ias 6: ج ج چ چ پڑ پڑ پڑ پڑ پڑ پڑ پڑ پڑ پڑ پڑ پڑ پڑ پڑ پڑ پڑ پڑ پڑ پڑ پڑ

###
ias ssad ہ ٹی sa ٹ bnaa ئیں ک ک ک ک ک ک ک ت ت ت ط ط ط ط ط ط ط ط ط
Create `simulation_exercises/test/simulation_test.py`:
```python
#!/usr/bin/env python3
import rclpy
سے rclpy.نود import نود
سے geometry_msgs.msg import Twist
سے nav_msgs.msg import Odometry
سے sensor_msgs.msg import LaserScan
import time

class SimulationTest:
    def __init__(self):
        super().__init__('simulation_test')
        
        # Create publishers اور subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Test state
        self.test_step = 0
        self.initial_pos = None
        self.scan_received = False
        
        # Create timer کے لیے test execution
        self.timer = self.create_timer(1.0, self.run_test)
        
        self.get_logger().info

    def odom_callback(self, msg):
        """Store initial position کے لیے comparison"""
        اگر self.initial_pos ہے None:
            self.initial_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def scan_callback(self, msg):
        """Confirm scan ہے ہوتے ہوئے received"""
        self.scan_received = True

    def run_test(self):
        """Run کا/کی test sequence"""
        اگر self.test_step == 0:
            self.get_logger().info('Test Step 0: Verifying sensor data...')
            اگر self.scan_received:
                self.get_logger().info
            else:
                self.get_logger().warn
            
            اگر self.initial_pos:
                self.get_logger().info(f'✓ Initial position: {self.initial_pos}')
            else:
                self.get_logger().warn
            
        elif self.test_step == 1:
            self.get_logger().info
            twist = Twist()
            twist.linear.x = 0.5  # Move forward پر 0.5 m/s
            self.cmd_vel_pub.publish(twist)
            
        elif self.test_step == 2:
            self.get_logger().info
            # Stop کا/کی روبوٹ
            twist = Twist()
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)
            
        elif self.test_step == 3:
            self.get_logger().info('Test Sequence Complete')
            self.timer.cancel()
            return
        
        self.test_step += 1

def main(args=None):
    rclpy.init(args=args)
    test_node = SimulationTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## vr ک ias 7: پیکیج tttt tat ف ف ف ف ف ف ف ف ف ف ف

### taa زہ ک ar ی پیکیج ف a ئ l یں
Update `simulation_exercises/پیکیج.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<پیکیج format="3">
  <name>simulation_exercises</name>
  <version>0.1.0</version>
  <description>سمولیشن exercises کے لیے روبوٹکس course</description>
  <maintainer email="user@مثال.com">User</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>xacro</depend>
  <depend>gazebo_ros_pkgs</depend>
  <depend>gazebo_plugins</depend>
  <depend>gazebo_dev</depend>
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  <depend>controller_manager</depend>
  <depend>diff_drive_controller</depend>

  <buildtool_depend>ament_python</buildtool_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</پیکیج>
```
Update `simulation_exercises/ترتیب.py`:
```python
سے setuptools import ترتیب
سے glob import glob
import os

package_name = 'simulation_exercises'

ترتیب(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include یوآر ڈی ایف files
, glob),
        # Include world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@مثال.com',
    description='سمولیشن exercises کے لیے روبوٹکس course',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = simulation_exercises.robot_controller:main',
            'sensor_processor = simulation_exercises.sensor_processor:main',
            'simulation_test = simulation_exercises.simulation_test:main',
        ],
    },
)
```
## vr ک ias 8: چ چ/کی/کی Mauml smul ہ n

### اعدعمت ک وفمال امل امل:

1.
```bash
cd ~/ros2_ws
colcon build --packages-select simulation_exercises
source install/ترتیب.bash
```
2.
```bash
ros2 launch simulation_exercises simulation_exercises.launch.py
```
3.
```bash
# کو visualize کا/کی روبوٹ model
ros2 run rviz2 rviz2

# کو check ٹاپکس
ros2 ٹاپک list

# کو see messages پر specific ٹاپکس
ros2 ٹاپک echo /odom
ros2 ٹاپک echo /scan
```
4.
```bash
ros2 run simulation_exercises simulation_test
```
5.
```bash
# Send velocity commands
ros2 ٹاپک pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.1}}'
```
## اعم مسعسل ک a aiaalہ ا ہ ہ ہ ہ ہ ثر

### 1

گ ک ک ک a/کی rewboc v کے کے کے کے aaaamat ک aawab ni ج j ج j ج ج ج ج ج ج ج ج ج ج ج ج ج:
```bash
# Check اگر کا/کی diff_drive_controller ہے running
ros2 لائف سائیکل list diff_drive_controller

# Check اگر robot_description ہے available
ros2 param list | grep robot_description

# Check کے لیے errors میں کا/کی diff_drive_controller
ros2 لائف سائیکل set diff_drive_controller configure
ros2 لائف سائیکل set diff_drive_controller activate
```
### 2

AASHR SS ی NSSR ٹ A پک S ہیں N ہیں ہ ہ ہ ہ ہ ے ہ vi vi iaaaaaad:
```bash
# Check اگر plugins ہیں loaded
gz model -m diff_drive_robot --info

# Check کے لیے گیزبو model plugins
ros2 run gazebo_ros spawn_entity.py -h
```
### 3. TF IDR خ atoch کے Massa ئ l

سٹر -
```bash
# View کا/کی TF tree
ros2 run tf2_tools view_frames

# Echo specific transforms
ros2 run tf2_ros tf2_echo odom base_link
```
## ک اراپراڈ گی کی کی ص ص laa ح کے کے n ک at

### 1

، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ط ، ، ، ، ، ، ، ، ، ط ط ، ، ، ، ، ، ، ، ط ط ، ، ، ، ، ، ، ، ، ط ط ، ، ، ، ، ، ، ، ط ط ط ، ، ، ، ، ، ، نیچے ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ط ط ط ، گی کی کی کی کی گی ، ، ، ط ط ط ، ، کی کی کی کی کی گی ،
```xml
<physics name="ode" default="0" type="ode">
  <max_step_size>0.01</max_step_size>  <!-- Increase کے لیے performance -->
  <real_time_factor>1.0</real_time_factor>
  <ode>
    <solver>
      <iters>20</iters>  <!-- Reduce کے لیے performance -->
      <sor>1.3</sor>
    </solver>
  </ode>
</physics>
```
### 2

سنسر کی TAA زہ ک ک ی کی کی ش ش ش ش J ش ش SR ح S ک ک S ک S ک S ک S ک ک ک L یے Biautr ISARAIRD گی:
```xml
<sensor name="camera" type="camera">
  <update_rate>15</update_rate>  <!-- Reduce کے لیے performance -->
</sensor>
```
## خ LAA صہ

عمال اعمت وِسر ِ ِ ِ ڈھ ی ی ی ی ی ی ی ی ی ڈھ

1.
2.
3.
4.
5.
6.
7

یہ mi شقیں ہ atauch ss ے ta ج rbi ک چوہا ی ہیں۔ آپ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ نیچے ھ ھ ھ ھ ہیں ہیں ہیں ہیں ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ کھ ھ ھ ھ ھ ھ ھ ھ ہیں ہیں ہیں ھ ھ ھ ھ ھ ھ ہیں ہیں ہیں ہیں ھ ھ ھ ھ ھ ہیں ہیں ہیں ہیں ھ ھ ھ ھ ھ ھ ہیں ہیں ہیں ہیں ھ ھ ھ ھ ھ ھ ھ ہیں ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ نیچے کھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ کھ کھ ھ ھ ھ ھ ھ ھ ھ کھ کھ ھ ھ ھ ھ ھ ھ ھ کھ کھ کھ ھ ھ ھ ھ ھ ھ کھ کھ کھ ھ ھ ھ ھ ھ کھ کھ کھ کھ کھ ھ ھ ھ ھ کھ کھ کھ کھ کھ کھ کھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ نیچے ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ نیچے ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ث ث ث ث ث
