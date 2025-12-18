---


sidebar_position: 3
difficulty: beginner


---
# 4.3: اونوومام ک a ٹی ٹی ٹی ٹی 2 اوسور ی ویسسر ڈی اوسسس ماماما ر روبوبو سیسسم

## ج a ج

یہ الل l ی ماماول نِن ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹ ٹ ٹ کے کے کے کے ہم کی s ے ی ی oaur ڈی ڈی ک/کی ج ahamd ڈھ ڈھanچہ فraaum ک ہیں ہیں ٹی ٹی ٹی ٹی فraum rroobws a یپ li jun ز ju ک so چ alv ک ک ک یں گے۔ گے۔

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- ssmau کہ s ے s ے ی oaur ڈی ڈی ڈی ڈی ڈی یف ٹی ٹی ٹی یف یف یف 2 یف 2 ia یک SAAT ھ ک aam isr یں
- mauml ط vr پ r پ روبو ما ڈ l بونا ئیں۔
- روبوو_س ٹیٹ_بل یش ک v buraausas ٹ آف braa ڈک sas ٹ ک a ساساتاہمال ال
- - ک saum ٹ raynsiaurm پblشr ک ک v ana فذ ک ri کے l یے l یے Matair ک
- ڈی BI گ ٹی ٹی کے کے کے کے کے کے د کے کے کے کے کے کے کے کے کے
- شامل کریں ہ ml ف ف ف ف ٹی ٹی ٹی ف ٹی ٹی ٹی ٹی ی ی ی کیش کیش کیش کیش کیش کیش ی ی کیش کیش jai ز jai ز jai ز jut

## تحم ک a/کی tf-- ی oaur ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی

### ج AAMD Bumahabl ہ Matair ک Jutbadal ی

ک a/کی ی oaur ڈی ڈی ڈی ڈی ڈی ڈی ٹی ٹی ٹی ٹی ٹی 2 کے Mababiantautahat atbadaul atbdaul ک ک ک ک ہے۔

1.
2. ذ ذ کے ذ ذ ذ ذ کے کے کے کے ذ ذ ذ کے-
```
یوآر ڈی ایف Structure -> robot_state_publisher -> Static Transforms
Joint States -> joint_state_publisher -> Dynamic Transforms
Both combined -> Complete TF Tree
```
### tf dri خ t tahar ک rd ہ ss ے ی ی ی oaur ڈی ia یف

ک آپ آپ آپ آپ آپ آپ آپ آپ آپ آپ آپ آپ آپ آپ آپ آپ آپ آپ ہ ہ ہ ڈی ڈی ڈی ڈی ک ک ک ک ڈی ہ ہ ہ ہ ک ک ک ک ک ک ک ڈی ڈی ہ ہ ڈی ہ ک ک ک ک آپ آپ ہ ہ ڈی ڈی ڈی ڈی ڈی آپ آپ آپ ہ ڈی ڈی ڈی ڈی ڈی آپ ف swauriuni ف ف swauriuni یٹ ف ف ف swauriun یٹ ف ک uauriun یٹ ف ف ف ف ف ف ف ف ں ں ں J خ J خ خ J خ خ خ J خ خ J خ خ J خ
```
base_link (root)
├── imu_link
├── left_wheel_link
├── right_wheel_link
├── camera_link
└── lidar_link
```
ہ ہ ہ ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ف ف کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے ج ج ج ج j j j j jah jutbd ی Lauch maus jutbd ی ی ی ی کی کی کی کی کی کی کی کی

## mauml ط vr پ rrubob maa ڈ l بونا

کیa

آئیے ، خ خ خ خ ک ک ک ک ک ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی
```xml
<?xml version="1.0"?>
<روبوٹ xmlns:xacro="http://www.ros.org/wiki/xacro" name="integrated_robot">
  
  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931"/>
  <xacro:property name="base_width" value="0.5"/>
  <xacro:property name="base_length" value="0.8"/>
  <xacro:property name="base_height" value="0.2"/>
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="wheel_mass" value="0.5"/>
  <xacro:property name="base_mass" value="20.0"/>
  
  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
  
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue"/>
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
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  
  <!-- Base footprint کے لیے 2D navigation -->
  <link name="base_footprint">
    <visual>
      <geometry>
        <cylinder radius="0.01" length="0.01"/>
      </geometry>
      <material name="white"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.3" length="0.1"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.0001"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  
  <!-- Joint between base footprint اور base link -->
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
        <material name="black"/>
      </visual>
      
      <collision>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      
      <inertial>
        <mass value="${wheel_mass}"/>
        <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
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
  <xacro:wheel prefix="front_left" reflect="1"/>
  <xacro:wheel prefix="front_right" reflect="-1"/>
  <xacro:wheel prefix="rear_left" reflect="1"/>
  <xacro:wheel prefix="rear_right" reflect="-1"/>
  
  <!-- Sensor mount -->
  <link name="sensor_mount_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Joint کو mount sensors -->
  <joint name="sensor_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_mount_link"/>
    <origin xyz="${base_length/2 - 0.05} 0 ${base_height/2 + 0.025}" rpy="0 0 0"/>
  </joint>
  
  <!-- Camera link -->
  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.1 0.03"/>
      </geometry>
      <material name="black"/>
    </visual>
    
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  
  <joint name="camera_joint" type="fixed">
    <parent link="sensor_mount_link"/>
    <child link="camera_link"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
  </joint>
  
  <!-- LIDAR link -->
  <link name="lidar_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0002"/>
    </inertial>
  </link>
  
  <joint name="lidar_joint" type="fixed">
    <parent link="sensor_mount_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
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
    <origin xyz="0 0 ${base_height/2 - 0.05}" rpy="0 0 0"/>
  </joint>
</روبوٹ>
```
## روبو ، re ی asat پ bl ش r anaumamam

### لن چ ٹال

ایب اللیں ک/کی llan چ ف aaul ک v bunaa ئیں - munassb ichr یقے ss ے ss ے sc ے c ک c یٹ ari یں ari یں assr // کی روبو رِسیسس:
```python
# launch/integrated_robot.launch.py
سے launch import LaunchDescription
سے launch.ایکشنز import DeclareLaunchArgument
سے launch.substitutions import کمانڈ, PathJoinSubstitution
سے launch_ros.ایکشنز import نود
سے launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='integrated_robot.یوآر ڈی ایف.xacro',
            description='یوآر ڈی ایف/XACRO description file کے ساتھ کا/کی روبوٹ'
        )
    )

    # Get یوآر ڈی ایف via xacro
    robot_description_content = کمانڈ(
        [
            PathJoinSubstitution([FindPackageShare("my_robot_description"), "یوآر ڈی ایف", "integrated_robot.یوآر ڈی ایف.xacro"])
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # روبوٹ State پبلشر
    robot_state_publisher_node = نود(
        پیکیج='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # Joint State پبلشر
    joint_state_publisher_node = نود(
        پیکیج='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'source_list': ['joint_states']
        }]
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            joint_state_publisher_node,
        ]
    )
```
## ماؤتری ری ی ست پ بل ش r پ r پ r Jlid دلد

## پھr

آئیے نووس واس ایس این ے پ پ پ پ پ پ پ ہ ہ ہ ہ ہ ہ ہ کہ ی ش ش کے کے کے کے کے کے کے کے کے کے کے ش ش ش ش ش کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے
```python
# robot_integration/robot_integration/joint_state_publisher.py
import rclpy
سے rclpy.نود import نود
سے sensor_msgs.msg import JointState
سے std_msgs.msg import Header
سے rclpy.qos import QoSProfile
import math

class IntegratedJointStatePublisher:
    def __init__(self):
        super().__init__('integrated_joint_state_publisher')
        
        # Create پبلشر کے لیے joint states
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        
        # Create timer کو publish joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20 Hz
        self.time = 0.0
        
        # Track جس joints ہیں continuous (wheels)
        self.wheel_joints = ['front_left_wheel_joint', 'front_right_wheel_joint', 
                            'rear_left_wheel_joint', 'rear_right_wheel_joint']

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
        
        # Set header
        msg.header = Header()
        msg.header.stamp = self.get_clock().اب().to_msg()
        msg.header.frame_id = 'joint_states'
        
        # Add wheel joint states کے ساتھ simulated motion
        کے لیے joint_name میں self.wheel_joints:
            msg.name.append(joint_name)
            
            # Simulate wheel rotation based پر time
            # میں ایک real روبوٹ, یہ کرے گا come سے encoders
            position = self.time * 2.0  # Rotate پر 2 rad/s
            اگر 'صحیح' میں joint_name:  # صحیح wheels go opposite direction کے لیے forward motion
                position = -position
                
            msg.position.append(position)
            msg.velocity.append(2.0)  # Constant velocity
            msg.effort.append(0.0)    # نہیں effort میں سمولیشن
        
        # میں ایک real روبوٹ, آپ'd also add other joints like:
        # - Arm joint positions سے encoders
        # - Gripper positions
        # - Head/swivel positions
        
        # Publish کا/کی message
        self.joint_pub.publish(msg)
        
        # Update time
        self.time += 0.05

def main(args=None):
    rclpy.init(args=args)
    نود = IntegratedJointStatePublisher()
    rclpy.spin
    نود.destroy_node()
    rclpy.shutdown()
```
## ٹی ia یف 2 جون ض مم مامال

### ی vaur ڈی ڈی ف ف raumwi ک a ک asatamal ک cus ہ sincsr asa ک se ک s

آئیے ، یک یک یک یک یک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک
```python
# robot_integration/robot_integration/sensor_transformer.py
import rclpy
سے rclpy.نود import نود
سے sensor_msgs.msg import LaserScan, PointCloud2
سے geometry_msgs.msg import PointStamped, TransformStamped
سے tf2_ros import TransformListener, Buffer
سے tf2_ros import TransformException
سے rclpy.qos import QoSProfile
import numpy کے طور پر np
import tf_transformations

class SensorTransformer:
    def __init__(self):
        super().__init__('sensor_transformer')
        
        # Create ایک transform buffer
        self.tf_buffer = Buffer()
        
        # Create ایک transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe کو laser scan
        qos = QoSProfile(depth=10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos)
        
        # Publish transformed scan
        self.transformed_scan_pub = self.create_publisher(
            LaserScan, 'scan_in_base_link', qos)
        
        # Publish specific point transformation مثال
        self.point_pub = self.create_publisher(
            PointStamped, 'transformed_point', qos)
        
        self.get_logger().info

    def scan_callback(self, msg):
        try:
            # Transform کا/کی laser scan سے lidar frame کو base_link frame
            transform = self.tf_buffer.lookup_transform(
                'base_link',      # Target frame
                msg.header.frame_id,  # Source frame
                msg.header.stamp,     # Time کا کا/کی scan
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            # میں ایک real نفاذ, آپ کرے گا transform each range reading
            # یہاں we just verify کا/کی transform exists اور log یہ
            self.get_logger().info(
                f'Scan frame {msg.header.frame_id} کو base_link - '
                f'transform available پر time {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
            
            # Create اور publish ایک transformed version کا کا/کی scan
            transformed_scan = msg
            transformed_scan.header.frame_id = 'base_link'
            self.transformed_scan_pub.publish(transformed_scan)
            
        except TransformException کے طور پر ex:
            self.get_logger().warn

    def transform_point_example(self):
        """مثال کا transforming ایک point between frames"""
        try:
            # Create ایک point میں کا/کی lidar frame
            point_in_lidar = PointStamped()
            point_in_lidar.header.frame_id = 'lidar_link'
            point_in_lidar.header.stamp = self.get_clock().اب().to_msg()
            point_in_lidar.point.x = 1.0  # 1 meter میں سامنے کا LIDAR
            point_in_lidar.point.y = 0.0
            point_in_lidar.point.z = 0.0
            
            # Transform کو base_link frame
            point_in_base = self.tf_buffer.transform(
                point_in_lidar,
                'base_link',
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Log کا/کی result
            self.get_logger().info(
                f'Point transformed: ({point_in_lidar.point.x:.2f}, '
                f'{point_in_lidar.point.y:.2f}, {point_in_lidar.point.z:.2f}) میں '
                f'{point_in_lidar.header.frame_id} -> '
                f'({point_in_base.point.x:.2f}, {point_in_base.point.y:.2f}, '
                f'{point_in_base.point.z:.2f}) میں {point_in_base.header.frame_id}')
            
            # Publish کا/کی transformed point
            self.point_pub.publish(point_in_base)
            
        except TransformException کے طور پر ex:
            self.get_logger().warn

def main(args=None):
    rclpy.init(args=args)
    نود = SensorTransformer()
    
    # Run کا/کی point transformation مثال periodically
    timer = نود.create_timer
    
    try:
        rclpy.spin
    except KeyboardInterrupt:
        نود.get_logger().info
    finally:
        timer.cancel()
        نود.destroy_node()
        rclpy.shutdown()
```
## اعل ی کی ٹی ِ ِ ِ s 2 ِ Sataamal کے SAAT ھ ی ی vaur ڈی Ai یف

### re ی asat ک a taumaun ہ Llulainے vla aِs rewbob کی

یہ ک ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی 2 کے 2 کے کے ھ ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ک ک ک ٹی ٹی ٹی ٹی ٹی ٹی ٹی ک ک ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ک ک ک ٹی ٹی ٹی ٹی ٹی ٹی ھ ھ ھ ھ ھ ھ ھ ھ
```python
# robot_integration/robot_integration/robot_state_estimator.py
import rclpy
سے rclpy.نود import نود
سے nav_msgs.msg import Odometry
سے geometry_msgs.msg import TransformStamped, Point, Pose, Quaternion
سے sensor_msgs.msg import Imu, LaserScan
سے tf2_ros import TransformBroadcaster
سے tf2_ros import TransformException
import tf_transformations
import math

class RobotStateEstimator:
    def __init__(self):
        super().__init__('robot_state_estimator')
        
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe کو sensor data
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # Timer کو broadcast transforms
        self.timer = self.create_timer(0.05, self.broadcast_transforms)  # 20 Hz
        
        # روبوٹ state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Store last timestamp کے لیے velocity calculation
        self.last_time = self.get_clock().اب()
        
        self.get_logger().info

    def imu_callback(self, msg):
        # Extract orientation سے IMU
        self.theta = self.quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        # Extract angular velocity
        self.angular_velocity = msg.angular_velocity.z

    def scan_callback(self, msg):
        # یہ callback کر سکتا تھا ہونا used کے لیے laser-based localization
        # کے لیے یہ مثال, we'll just log کا/کی fact وہ we received ایک scan
        pass

    def odom_callback(self, msg):
        # Update position سے odometry
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Update orientation
        self.theta = self.quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        
        # Update velocities
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion کو yaw angle"""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def broadcast_transforms(self):
        # Broadcast کا/کی transform سے odom کو base_footprint
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().اب().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert theta کو quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Send کا/کی transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    نود = RobotStateEstimator()
    rclpy.spin
    نود.destroy_node()
    rclpy.shutdown()
```
## ٹی ٹی یف یف 2 اوسور اوسر ڈی ڈی یف یف یف ڈی ڈی baun گ

### tf idr خ atoau کی Tauchr ی t جزیہ

آئیے iaur ڈی b گ -/کی ٹی ٹی ٹی ٹی ٹی ٹی ٹی a ک a ک a tai jurn ے jurn ے jumad jdd ک ri:
```python
# robot_integration/robot_integration/tf_analyzer.py
import rclpy
سے rclpy.نود import نود
سے tf2_msgs.msg import TFMessage
سے tf2_ros import Buffer, TransformListener
سے rclpy.qos import QoSProfile
import time

class TFAgent:
    def __init__(self):
        super().__init__('tf_analyzer')
        
        # Create TF buffer اور listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create timer کو periodically analyze TF tree
        self.timer = self.create_timer(5.0, self.analyze_tf_tree)
        
        self.get_logger().info('TF Analyzer started')

    def analyze_tf_tree(self):
        """Analyze کا/کی current TF tree structure"""
        try:
            # Get تمام available transforms
            transforms = self.tf_buffer.all_frames_as_yaml()
            self.get_logger().info(f'Available TF frames:\n{transforms}')
            
            # Check specific transforms وہ چاہیے exist
            required_transforms = [
                ('base_link', 'base_footprint'),
                ('base_link', 'lidar_link'),
                ('base_link', 'camera_link'),
                ('base_link', 'imu_link'),
                ('base_link', 'front_left_wheel_link'),
                ('base_link', 'front_right_wheel_link'),
                ('base_link', 'rear_left_wheel_link'),
                ('base_link', 'rear_right_wheel_link'),
            ]
            
            missing_transforms = []
            available_transforms = []
            
            کے لیے parent, child میں required_transforms:
                try:
                    self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                    available_transforms.append(f'{parent} -> {child}')
                except Exception:
                    missing_transforms.append(f'{parent} -> {child}')
            
            اگر available_transforms:
                self.get_logger().info(f'Available transforms: {", ".join(available_transforms)}')
            
            اگر missing_transforms:
                self.get_logger().warn(f'Missing transforms: {", ".join(missing_transforms)}')
            else:
                self.get_logger().info
        
        except Exception کے طور پر e:
            self.get_logger().error(f'Error analyzing TF tree: {e}')

def main(args=None):
    rclpy.init(args=args)
    نود = TFAgent()
    rclpy.spin
    نود.destroy_node()
    rclpy.shutdown()
```
## عملی وورک اِسسس: ماممل اعور اعنیحمام

###
```python
# launch/complete_robot_system.launch.py
سے launch import LaunchDescription
سے launch.ایکشنز import DeclareLaunchArgument, IncludeLaunchDescription
سے launch.launch_description_sources import PythonLaunchDescriptionSource
سے launch.substitutions import کمانڈ, PathJoinSubstitution
سے launch_ros.ایکشنز import نود
سے launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    description_file_arg = DeclareLaunchArgument(
        'description_file',
        default_value='integrated_robot.یوآر ڈی ایف.xacro',
        description='یوآر ڈی ایف/XACRO description file کے ساتھ کا/کی روبوٹ'
    )
    
    # Get یوآر ڈی ایف via xacro
    robot_description_content = کمانڈ(
        [
            PathJoinSubstitution([FindPackageShare("my_robot_description"), "یوآر ڈی ایف", "integrated_robot.یوآر ڈی ایف.xacro"])
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # روبوٹ State پبلشر
    robot_state_publisher_node = نود(
        پیکیج='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # Joint State پبلشر
    joint_state_publisher_node = نود(
        پیکیج='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'source_list': ['joint_states']
        }]
    )

    # Custom joint state پبلشر
    custom_joint_publisher_node = نود(
        پیکیج='my_robot_integration',
        executable='joint_state_publisher',
        name='custom_joint_publisher',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    # Sensor transformer
    sensor_transformer_node = نود(
        پیکیج='my_robot_integration',
        executable='sensor_transformer',
        name='sensor_transformer',
    )

    # روبوٹ state estimator
    state_estimator_node = نود(
        پیکیج='my_robot_integration',
        executable='robot_state_estimator',
        name='robot_state_estimator',
    )

    # TF analyzer
    tf_analyzer_node = نود(
        پیکیج='my_robot_integration',
        executable='tf_analyzer',
        name='tf_analyzer',
    )

    return LaunchDescription([
        description_file_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        custom_joint_publisher_node,
        sensor_transformer_node,
        state_estimator_node,
        tf_analyzer_node,
    ])
```
## mautri کہ Jaumamam کے کے کے کے کے کے ح ح ح ح ح ح ح ح ح ح

### 1
```python
# Problem: Sensor data رکھتا ہے غلط frame_id
# Solution: Always check اور validate frame IDs

class RobustSensorProcessor:
    def __init__(self):
        super().__init__('robust_sensor_processor')
        self.valid_frames = set()
        
    def sensor_callback(self, msg):
        اگر msg.header.frame_id نہیں میں self.valid_frames:
            # Check اگر کا/کی frame exists میں TF
            try:
                # یہ کرے گا raise ایک exception اگر کا/کی frame doesn't exist
                self.tf_buffer.lookup_transform(
                    'base_link', msg.header.frame_id, rclpy.time.Time())
                self.valid_frames.add(msg.header.frame_id)
            except:
                self.get_logger().warn(f'Invalid frame_id: {msg.header.frame_id}')
                return
```
### 2۔ واٹ کی ہ ہ ہ m آہ n گی کے کے کے کے کے کے کے کے کے کے کے
```python
# Problem: Transforms پر غلط time
# Solution: Use proper time synchronization

سے rclpy.duration import Duration

class TimeSyncedProcessor:
    def __init__(self):
        super().__init__('time_synced_processor')
        
    def process_sensor_data(self, sensor_msg):
        try:
            # Add ایک چھوٹا buffer کو account کے لیے processing delay
            transform_time = sensor_msg.header.stamp
            transform_time.sec -= 1  # Look پیچھے میں time اگر needed
            
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                sensor_msg.header.frame_id,
                transform_time,
                timeout=Duration(seconds=0.1))
                
            # Process کے ساتھ transform
            return transform
        except Exception کے طور پر e:
            self.get_logger().error(f'Time sync error: {e}')
            return None
```
### 3. TF DR خ at ک v Itm Jurna
```python
# Problem: Disconnected frames میں TF tree
# Solution: Ensure تمام frames connect کو ایک common base

class TFContinuityChecker:
    def __init__(self):
        super().__init__('tf_continuity_checker')
        
    def check_continuity(self, target_frame, base_frame='base_link'):
        """Check اگر وہاں's ایک transform path between دو frames"""
        try:
            # یہ کرے گا fail اگر وہاں's نہیں path between frames
            self.tf_buffer.lookup_transform(
                base_frame, target_frame, rclpy.time.Time())
            return True
        except:
            return False
```
## ک اراورڈ گی کی کی ص ص ص ص ص کی

### MWAUR TF astamail
```python
class OptimizedTFNode:
    def __init__(self):
        super().__init__('optimized_tf_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Cache frequently accessed transforms
        self.cached_transforms = {}
        self.cache_timeout = 0.1  # 100ms cache
        
    def get_cached_transform(self, target_frame, source_frame):
        cache_key = f"{target_frame}_{source_frame}"
        
        # Check اگر cached اور نہیں expired
        اگر cache_key میں self.cached_transforms:
            transform, timestamp = self.cached_transforms[cache_key]
            current_time = self.get_clock().اب()
            
            اگر (current_time - timestamp).nanoseconds < self.cache_timeout * 1e9:
                return transform
        
        # Get fresh transform
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            
            # Cache یہ
            self.cached_transforms[cache_key] = (transform, self.get_clock().اب())
            return transform
        except Exception کے طور پر e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return None
```
## خ LAA صہ

یہ ایلل لِل ، ماما ماامال نِن ک/کی کے کے کے کے j j کے j j j کے

1.
2.
3.
4.
5.
6.

ک a/کی احامام ک a ٹی ai ٹی 2 اوسور ی vaur ڈی ہے binیadai ک binیadai ک bunیadai jurws rwosossoss پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی mulٹی jraumrubouٹک nahamwaں کs ذی ذی ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ، ہ ہ ہف ہف ہف ہف ہف ہف ہف 4 ، ہ ہ ہ یہ یہ یہ ہف ہف ہف ہف ، ہ ہ یہ یہ یہ ہف ہف ہف ہف ہف ہ ہ ہ ہف ہف ہف ہف ہف ہف ہف ہف ہ ہ ہف ہف ہف ہف ہف ہف ہف ہف ہف کے کے ہف ہف ہف ہف ہف ہف ہف ہف کے کے کے
