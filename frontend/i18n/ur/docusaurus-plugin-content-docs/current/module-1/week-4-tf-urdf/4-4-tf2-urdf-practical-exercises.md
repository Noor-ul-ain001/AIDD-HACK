---


sidebar_position: 4
difficulty: beginner


---
# 4.4: امتی کی کی کی ٹی ٹی - ٹی اِس 2 اواور ووور ی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی

## ج a ج

یہ الل لِل ، نِن ِ ہ ہ کی ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- ی ی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے ھ کے کے کے کے کے
- اِنوومام ٹی ٹی یف 2 کے saaٹ ھ rewbo ٹ reausasat کی
- عمل ia ia یف 2 ک a ک amaal ک samal ک ہ vi ہ cel ی ssincsr ڈیٹ a ک s
- ڈی b گ ٹی ٹی کے کے کے د کے کے کے کے کے کے کے کے ک ک ک Maul یک s rewbo sssausm
- مکمل نیویگیشن کے لئے تیار روبوب ماڈل بنائیں
- ک اراپرڈ گی کی a ص laa ح کی کی کی کی یک ک a a ط laa ق ک ri یں

## vr ک JAS 1: اومرات یک maumul jur پ vr jrubo

###
maus یک mauml Assr ڈی Ass maa ڈ l Buna ئیں۔

### mracl ہ 1: ک a/کی پیکیج پیکیج پیکیج ڈھ ڈھ چہ چہ چہ چہ چہ چہ ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک

الا ، تالیق اِل
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_model_exercises
```
### mrapl ہ 2: ک a/کی ی ی vaur ڈی ڈی maaul Buna ئیں
Create کا/کی file `robot_model_exercises/یوآر ڈی ایف/exercise_robot.یوآر ڈی ایف.xacro`:
```xml
<?xml version="1.0"?>
<روبوٹ xmlns:xacro="http://www.ros.org/wiki/xacro" name="exercise_robot">
  
  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931"/>
  <xacro:property name="base_width" value="0.6"/>
  <xacro:property name="base_length" value="0.8"/>
  <xacro:property name="base_height" value="0.3"/>
  <xacro:property name="wheel_radius" value="0.15"/>
  <xacro:property name="wheel_width" value="0.08"/>
  <xacro:property name="wheel_mass" value="2.0"/>
  <xacro:property name="base_mass" value="50.0"/>
  <xacro:property name="caster_radius" value="0.05"/>
  
  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
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
      <inertia ixx="5.0" ixy="0.0" ixz="0.0" iyy="5.0" iyz="0.0" izz="10.0"/>
    </inertial>
  </link>
  
  <!-- Base footprint -->
  <link name="base_footprint">
    <visual>
      <geometry>
        <cylinder radius="0.01" length="0.001"/>
      </geometry>
      <material name="white"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.01" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${base_width/2}" length="0.02"/>
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
        <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.2"/>
      </inertial>
    </link>
    
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel_link"/>
      <origin xyz="${base_length/2 - wheel_width/2} ${reflect * (base_width/2 + wheel_width/2)} -${base_height/2}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>
  
  <!-- Create wheels -->
  <xacro:wheel prefix="front_left" reflect="1"/>
  <xacro:wheel prefix="front_right" reflect="-1"/>
  <xacro:wheel prefix="rear_left" reflect="1"/>
  <xacro:wheel prefix="rear_right" reflect="-1"/>
  
  <!-- سامنے caster wheel -->
  <link name="front_caster_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
      <material name="black"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="front_caster_joint" type="continuous">
    <parent link="base_link"/>
    <child link="front_caster_link"/>
    <origin xyz="${base_length/2 - caster_radius} 0 -${base_height/2 + caster_radius}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
  <!-- Rear caster wheel -->
  <link name="rear_caster_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
      <material name="black"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="rear_caster_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_caster_link"/>
    <origin xyz="-${base_length/2 + caster_radius} 0 -${base_height/2 + caster_radius}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
  <!-- Sensor mount plate -->
  <link name="sensor_mount_plate">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.02"/>
      </geometry>
      <material name="white"/>
    </visual>
    
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>
  
  <joint name="sensor_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_mount_plate"/>
    <origin xyz="${base_length/2 - 0.1} 0 ${base_height/2 + 0.01}" rpy="0 0 0"/>
  </joint>
  
  <!-- RGB-D Camera -->
  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.1 0.03"/>
      </geometry>
      <material name="black"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.1 0.03"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  
  <joint name="camera_joint" type="fixed">
    <parent link="sensor_mount_plate"/>
    <child link="camera_link"/>
    <origin xyz="0.025 0 0.025" rpy="0 0 0"/>
  </joint>
  
  <!-- LIDAR -->
  <link name="lidar_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0002"/>
    </inertial>
  </link>
  
  <joint name="lidar_joint" type="fixed">
    <parent link="sensor_mount_plate"/>
    <child link="lidar_link"/>
    <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
  </joint>
  
  <!-- IMU -->
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
  
</روبوٹ>
```
### mriqul ہ 3: ai پ ڈیٹ پیکیج پیکیج ف ف aaul یں
Update `robot_model_exercises/پیکیج.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<پیکیج format="3">
  <name>robot_model_exercises</name>
  <version>0.1.0</version>
  <description>عملی exercises کے لیے ٹی ایف 2 اور یوآر ڈی ایف integration</description>
  <maintainer email="user@مثال.com">User</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>tf_transformations</depend>

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
Update `robot_model_exercises/ترتیب.py`:
```python
سے setuptools import ترتیب
سے glob import glob
import os

package_name = 'robot_model_exercises'

ترتیب(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include یوآر ڈی ایف files
, glob),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@مثال.com',
    description='عملی exercises کے لیے ٹی ایف 2 اور یوآر ڈی ایف integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_transformer = robot_model_exercises.sensor_transformer:main',
            'tf_visualizer = robot_model_exercises.tf_visualizer:main',
            'robot_state_publisher = robot_model_exercises.robot_state_publisher:main',
        ],
    },
)
```
## vr ک iass 2: سنوسر ڈیٹ کی کی کی کی کی کی ت کی کی کی کی کی کی کی کی کی کی کی کی

###
یک nwau vauso sbsauraiaub Buna ک ک sosunsr ک asa arsa ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی 2 ک asastaamal surtai surtai surtai surtai suratis ہ uaucuchuriushumuauchuchuchuchuchuratis aucuchumuauchuraturatis aochucuchurathuratis aocuchumuauchuratu
Create `robot_model_exercises/robot_model_exercises/sensor_transformer.py`:
```python
import rclpy
سے rclpy.نود import نود
سے sensor_msgs.msg import LaserScan, PointCloud2
سے geometry_msgs.msg import PointStamped, TransformStamped
سے tf2_ros import TransformListener, Buffer
سے tf2_ros import TransformException
سے tf2_geometry_msgs import do_transform_points
سے rclpy.qos import QoSProfile
import numpy کے طور پر np
import math
سے sensor_msgs_py import point_cloud2
سے sensor_msgs.msg import PointField

class SensorTransformer:
    def __init__(self):
        super().__init__('sensor_transformer')
        
        # Create ایک transform buffer
        self.tf_buffer = Buffer()
        
        # Create ایک transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create QoS profile
        qos = QoSProfile(depth=10)
        
        # Subscribe کو laser scan
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos)
        
        # Subscribe کو point cloud
        self.pc_sub = self.create_subscription(
            PointCloud2, 'point_cloud', self.point_cloud_callback, qos)
        
        # Publishers کے لیے transformed data
        self.transformed_scan_pub = self.create_publisher(
            LaserScan, 'scan_in_base_link', qos)
        self.transformed_point_pub = self.create_publisher(
            PointStamped, 'transformed_point', qos)
        
        # Timer کے لیے periodic frame checking
        self.timer = self.create_timer(1.0, self.check_frames)
        
        self.get_logger().info

    def scan_callback(self, msg):
        """Transform laser scan سے sensor frame کو base_link frame"""
        try:
            # Look اوپر transform سے lidar frame کو base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',      # Target frame
                msg.header.frame_id,  # Source frame
                msg.header.stamp,     # Time کا کا/کی scan
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Transform کا/کی laser scan
            transformed_scan = LaserScan()
            transformed_scan.header.stamp = self.get_clock().اب().to_msg()
            transformed_scan.header.frame_id = 'base_link'
            transformed_scan.angle_min = msg.angle_min
            transformed_scan.angle_max = msg.angle_max
            transformed_scan.angle_increment = msg.angle_increment
            transformed_scan.time_increment = msg.time_increment
            transformed_scan.scan_time = msg.scan_time
            transformed_scan.range_min = msg.range_min
            transformed_scan.range_max = msg.range_max
            transformed_scan.ranges = msg.ranges
            transformed_scan.intensities = msg.intensities اگر msg.intensities else []
            
            # Publish کا/کی transformed scan
            self.transformed_scan_pub.publish(transformed_scan)
            
            # Log کا/کی transformation
            self.get_logger().info(
                f'Transformed scan سے {msg.header.frame_id} کو base_link')
            
        except TransformException کے طور پر ex:
            self.get_logger().warn

    def point_cloud_callback(self, msg):
        """Transform point cloud سے sensor frame کو base_link frame"""
        try:
            # Look اوپر transform
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Extract points سے PointCloud2
            points = []
            کے لیے point میں point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2], 1.0])  # Format کے لیے transformation
            
            # Convert کو homogeneous coordinates اگر needed
            points_array = np.array(points).T
            
            # Create transformation matrix سے TF transform
            t = transform.transform.translation
            r = transform.transform.rotation
            
            # Create 4x4 transformation matrix
            transform_matrix = np.eye(4)
            # Translation
            transform_matrix[0, 3] = t.x
            transform_matrix[1, 3] = t.y
            transform_matrix[2, 3] = t.z
            
            # Rotation
            qw, qx, qy, qz = r.w, r.x, r.y, r.z
            # Convert quaternion کو rotation matrix
            transform_matrix[0, 0] = 1 - 2*(qy*qy + qz*qz)
            transform_matrix[0, 1] = 2*(qx*qy - qw*qz)
            transform_matrix[0, 2] = 2*(qx*qz + qw*qy)
            transform_matrix[1, 0] = 2*(qx*qy + qw*qz)
            transform_matrix[1, 1] = 1 - 2*(qx*qx + qz*qz)
            transform_matrix[1, 2] = 2*(qy*qz - qw*qx)
            transform_matrix[2, 0] = 2*(qx*qz - qw*qy)
            transform_matrix[2, 1] = 2*(qy*qz + qw*qx)
            transform_matrix[2, 2] = 1 - 2*(qx*qx + qy*qy)
            
            # Transform points
            transformed_points = transform_matrix @ points_array
            
            # Log transformation result
            self.get_logger().info(
                f'Transformed point cloud کے ساتھ {len(points)} points سے {msg.header.frame_id} کو base_link')
            
        except TransformException کے طور پر ex:
            self.get_logger().warn
        except Exception کے طور پر e:
            self.get_logger().warn(f'Point cloud transformation failed: {e}')

    def check_frames(self):
        """Periodically check اگر تمام required frames exist"""
        required_frames = [
            'base_link', 'base_footprint', 'lidar_link', 
            'camera_link', 'imu_link'
        ]
        
        missing_frames = []
        کے لیے frame میں required_frames:
            try:
                # Check اگر frame exists کے ذریعے looking اوپر transform کو itself میں کا/کی past
                self.tf_buffer.lookup_transform(frame, frame, rclpy.time.Time(), 
                                               timeout=rclpy.duration.Duration(seconds=0.1))
            except:
                missing_frames.append(frame)
        
        اگر missing_frames:
            self.get_logger().warn(f'Missing frames: {missing_frames}')
        else:
            self.get_logger().info

def main(args=None):
    rclpy.init(args=args)
    نود = SensorTransformer()
    
    try:
        rclpy.spin
    except KeyboardInterrupt:
        نود.get_logger().info
    finally:
        نود.destroy_node()
        rclpy.shutdown()
```
## وورک ایسس 3: ٹی ایسس وِنلالسن اوسروس
Create `robot_model_exercises/robot_model_exercises/tf_visualizer.py`:
```python
import rclpy
سے rclpy.نود import نود
سے visualization_msgs.msg import Marker, MarkerArray
سے geometry_msgs.msg import Point
سے tf2_ros import Buffer, TransformListener
سے tf2_ros import TransformException
سے rclpy.qos import QoSProfile
import math

class TFVisualizer:
    def __init__(self):
        super().__init__('tf_visualizer')
        
        # Create TF buffer اور listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create پبلشر کے لیے visualization markers
        qos = QoSProfile(depth=10)
        self.marker_pub = self.create_publisher(MarkerArray, 'tf_visualization_markers', qos)
        
        # Timer کو periodically update visualization
        self.timer = self.create_timer(0.5, self.update_visualization)
        
        self.get_logger().info('TF Visualizer started')

    def update_visualization(self):
        """Update visualization کا TF tree"""
        try:
            # Get تمام available frames
            all_frames = self.tf_buffer.all_frames_as_string()
            
            # Create marker array
            marker_array = MarkerArray()
            
            # Create ایک marker کے لیے each transform
            frame_names = [name.split(' ')[0] کے لیے name میں all_frames.split('\n') اگر name]
            frame_names = [name کے لیے name میں frame_names اگر name]  # Remove empty strings
            
            # Create axes کے لیے each frame
            کے لیے میں, frame میں enumerate(frame_names):
                try:
                    # Get transform سے base_link کو یہ frame
                    t = self.tf_buffer.lookup_transform(
                        'base_link', frame, rclpy.time.Time())
                    
                    # Create axis markers
                    self.add_frame_axes
                except TransformException:
                    # اگر کر سکتا ہے't transform کو base_link, skip یہ frame
                    continue
            
            # Publish کا/کی marker array
            self.marker_pub.publish(marker_array)
            
        except Exception کے طور پر e:
            self.get_logger().warn(f'Error updating visualization: {e}')

    def add_frame_axes(self, marker_array, transform, frame_name, frame_id):
        """Add coordinate axes visualization کے لیے ایک frame"""
        # X-axis (red)
        x_axis_marker = Marker()
        x_axis_marker.header.frame_id = 'base_link'
        x_axis_marker.header.stamp = self.get_clock().اب().to_msg()
        x_axis_marker.ns = f"x_axis_{frame_name}"
        x_axis_marker.id = frame_id * 3
        x_axis_marker.type = Marker.ARROW
        x_axis_marker.ایکشن = Marker.ADD
        
        # Set start point کو transform origin
        start_point = Point()
        start_point.x = transform.transform.translation.x
        start_point.y = transform.transform.translation.y
        start_point.z = transform.transform.translation.z
        
        # Set اختتام point کو ہونا along x-axis
        end_point = Point()
        end_point.x = start_point.x + 0.1  # 10cm along x-axis
        end_point.y = start_point.y
        end_point.z = start_point.z
        
        x_axis_marker.points = [start_point, end_point]
        x_axis_marker.scale.x = 0.01  # Shaft diameter
        x_axis_marker.scale.y = 0.02  # Head diameter
        x_axis_marker.color.r = 1.0
        x_axis_marker.color.g = 0.0
        x_axis_marker.color.b = 0.0
        x_axis_marker.color.ایک = 1.0
        
        marker_array.markers.append(x_axis_marker)
        
        # Y-axis (green)
        y_axis_marker = Marker()
        y_axis_marker.header.frame_id = 'base_link'
        y_axis_marker.header.stamp = self.get_clock().اب().to_msg()
        y_axis_marker.ns = f"y_axis_{frame_name}"
        y_axis_marker.id = frame_id * 3 + 1
        y_axis_marker.type = Marker.ARROW
        y_axis_marker.ایکشن = Marker.ADD
        
        start_point = Point()
        start_point.x = transform.transform.translation.x
        start_point.y = transform.transform.translation.y
        start_point.z = transform.transform.translation.z
        
        end_point = Point()
        end_point.x = start_point.x
        end_point.y = start_point.y + 0.1  # 10cm along y-axis
        end_point.z = start_point.z
        
        y_axis_marker.points = [start_point, end_point]
        y_axis_marker.scale.x = 0.01
        y_axis_marker.scale.y = 0.02
        y_axis_marker.color.r = 0.0
        y_axis_marker.color.g = 1.0
        y_axis_marker.color.b = 0.0
        y_axis_marker.color.ایک = 1.0
        
        marker_array.markers.append(y_axis_marker)
        
        # Z-axis (blue)
        z_axis_marker = Marker()
        z_axis_marker.header.frame_id = 'base_link'
        z_axis_marker.header.stamp = self.get_clock().اب().to_msg()
        z_axis_marker.ns = f"z_axis_{frame_name}"
        z_axis_marker.id = frame_id * 3 + 2
        z_axis_marker.type = Marker.ARROW
        z_axis_marker.ایکشن = Marker.ADD
        
        start_point = Point()
        start_point.x = transform.transform.translation.x
        start_point.y = transform.transform.translation.y
        start_point.z = transform.transform.translation.z
        
        end_point = Point()
        end_point.x = start_point.x
        end_point.y = start_point.y
        end_point.z = start_point.z + 0.1  # 10cm along z-axis
        
        z_axis_marker.points = [start_point, end_point]
        z_axis_marker.scale.x = 0.01
        z_axis_marker.scale.y = 0.02
        z_axis_marker.color.r = 0.0
        z_axis_marker.color.g = 0.0
        z_axis_marker.color.b = 1.0
        z_axis_marker.color.ایک = 1.0
        
        marker_array.markers.append(z_axis_marker)

    def get_tf_tree_info(self):
        """Get information about کا/کی TF tree structure"""
        try:
            tree_info = self.tf_buffer.all_frames_as_yaml()
            self.get_logger().info(f'TF Tree:\n{tree_info}')
            
            # Identify root frame
            all_frames = self.tf_buffer.all_frames_as_string()
            self.get_logger().info(f'Available frames: {all_frames}')
            
        except Exception کے طور پر e:
            self.get_logger().error(f'Error getting TF tree info: {e}')

def main(args=None):
    rclpy.init(args=args)
    نود = TFVisualizer()
    
    # Run tree info once پر startup
    نود.get_tf_tree_info()
    
    try:
        rclpy.spin
    except KeyboardInterrupt:
        نود.get_logger().info
    finally:
        نود.destroy_node()
        rclpy.shutdown()
```
## vr ک jais 4: روبوبی ی یsiٹیٹ پبلیسسر کے ستیی ھ سنوسر اعماما
Create `robot_model_exercises/robot_model_exercises/robot_state_publisher.py`:
```python
import rclpy
سے rclpy.نود import نود
سے sensor_msgs.msg import JointState
سے nav_msgs.msg import Odometry
سے geometry_msgs.msg import TransformStamped
سے tf2_ros import TransformBroadcaster
سے std_msgs.msg import Header
سے rclpy.qos import QoSProfile
import math
import numpy کے طور پر np

class ExerciseRobotStatePublisher:
    def __init__(self):
        super().__init__('exercise_robot_state_publisher')
        
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create QoS profile
        qos = QoSProfile(depth=10)
        
        # Subscribe کو joint states
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, qos)
        
        # Subscribe کو odometry
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos)
        
        # Publishers کے لیے joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos)
        
        # Timer کے لیے publishing state
        self.timer = self.create_timer(0.02, self.publish_robot_states)  # 50 Hz
        
        # روبوٹ state variables
        self.wheel_positions = {
            'front_left_wheel_joint': 0.0,
            'front_right_wheel_joint': 0.0,
            'rear_left_wheel_joint': 0.0,
            'rear_right_wheel_joint': 0.0
        }
        
        self.caster_positions = {
            'front_caster_joint': 0.0,
            'rear_caster_joint': 0.0
        }
        
        # روبوٹ pose سے odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Simulate روبوٹ movement
        self.sim_time = 0.0
        
        self.get_logger().info

    def joint_callback(self, msg):
        """Update joint positions سے joint state messages"""
        کے لیے میں, name میں enumerate(msg.name):
            اگر name میں self.wheel_positions:
                self.wheel_positions[name] = msg.position[میں]
            elif name میں self.caster_positions:
                self.caster_positions[name] = msg.position[میں]

    def odom_callback(self, msg):
        """Update روبوٹ pose سے odometry"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Convert quaternion کو yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def publish_robot_states(self):
        """Publish روبوٹ states including transforms"""
        # Simulate wheel motion
        self.sim_time += 0.02  # Increment سمولیشن time
        
        # Update wheel positions based پر simulated motion
        wheel_velocity = 1.0  # rad/s
        کے لیے joint_name میں self.wheel_positions:
            self.wheel_positions[joint_name] += wheel_velocity * 0.02
            اگر 'صحیح' میں joint_name:  # صحیح wheels rotate opposite کے لیے forward motion
                self.wheel_positions[joint_name] -= 2 * wheel_velocity * 0.02
        
        # Update caster positions
        کے لیے joint_name میں self.caster_positions:
            self.caster_positions[joint_name] = math.sin(self.sim_time) * 0.5  # Oscillating
        
        # Move روبوٹ میں ایک square pattern
        linear_vel = 0.5  # m/s
        current_seg = int(self.sim_time / 5) % 4  # 5 seconds per side
        seg_time = self.sim_time % 5
        
        اگر current_seg == 0:  # Moving میں +X direction
            self.x += linear_vel * 0.02
        elif current_seg == 1:  # Moving میں +Y direction
            self.y += linear_vel * 0.02
        elif current_seg == 2:  # Moving میں -X direction
            self.x -= linear_vel * 0.02
        else:  # Moving میں -Y direction
            self.y -= linear_vel * 0.02
        
        # Publish joint states
        self.publish_joint_states()
        
        # Publish transforms
        self.publish_transforms()

    def publish_joint_states(self):
        """Publish joint state messages"""
        msg = JointState()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
        
        msg.header = Header()
        msg.header.stamp = self.get_clock().اب().to_msg()
        msg.header.frame_id = 'joint_states'
        
        # Add wheel joints
        کے لیے joint_name, position میں self.wheel_positions.items():
            msg.name.append(joint_name)
            msg.position.append(position)
            msg.velocity.append(1.0)  # Simulated velocity
            msg.effort.append(0.0)
        
        # Add caster joints
        کے لیے joint_name, position میں self.caster_positions.items():
            msg.name.append(joint_name)
            msg.position.append(position)
            msg.velocity.append(0.5)  # Simulated velocity
            msg.effort.append(0.0)
        
        self.joint_pub.publish(msg)

    def publish_transforms(self):
        """Publish تمام static اور dynamic transforms"""
        # Transform سے odom کو base_footprint (simulating odometry)
        t = TransformStamped()
        t.header.stamp = self.get_clock().اب().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert theta کو quaternion
        سے tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        # Transform سے base_footprint کو base_link (fixed offset)
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().اب().to_msg()
        t2.header.frame_id = 'base_footprint'
        t2.child_frame_id = 'base_link'
        
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.15  # base_height/2 = 0.3/2
        
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t2)
        
        # Transform کے لیے each wheel
        wheel_transforms = [
            ('front_left_wheel_joint', 'base_link', 'front_left_wheel_link', 0.35, 0.35, -0.15),
            ('front_right_wheel_joint', 'base_link', 'front_right_wheel_link', 0.35, -0.35, -0.15),
            ('rear_left_wheel_joint', 'base_link', 'rear_left_wheel_link', -0.35, 0.35, -0.15),
            ('rear_right_wheel_joint', 'base_link', 'rear_right_wheel_link', -0.35, -0.35, -0.15),
        ]
        
        کے لیے joint_name, parent_frame, child_frame, x, y, z میں wheel_transforms:
            t = TransformStamped()
            t.header.stamp = self.get_clock().اب().to_msg()
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            
            # Rotate wheel based پر joint position
            q_wheel = quaternion_from_euler(0, 0, self.wheel_positions[joint_name])
            t.transform.rotation.x = q_wheel[0]
            t.transform.rotation.y = q_wheel[1]
            t.transform.rotation.z = q_wheel[2]
            t.transform.rotation.w = q_wheel[3]
            
            self.tf_broadcaster.sendTransform(t)
        
        # Transform کے لیے caster wheels
        caster_transforms = [
            ('front_caster_joint', 'base_link', 'front_caster_link', 0.25, 0.0, -0.15),
            ('rear_caster_joint', 'base_link', 'rear_caster_link', -0.25, 0.0, -0.15),
        ]
        
        کے لیے joint_name, parent_frame, child_frame, x, y, z میں caster_transforms:
            t = TransformStamped()
            t.header.stamp = self.get_clock().اب().to_msg()
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            
            # Rotate caster based پر joint position
            q_caster = quaternion_from_euler(
                math.sin(self.caster_positions[joint_name]) * 0.1,
                math.cos(self.caster_positions[joint_name]) * 0.1,
                self.caster_positions[joint_name]
            )
            t.transform.rotation.x = q_caster[0]
            t.transform.rotation.y = q_caster[1]
            t.transform.rotation.z = q_caster[2]
            t.transform.rotation.w = q_caster[3]
            
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    نود = ExerciseRobotStatePublisher()
    
    try:
        rclpy.spin
    except KeyboardInterrupt:
        نود.get_logger().info
    finally:
        نود.destroy_node()
        rclpy.shutdown()
```
## وورک ایسس 5: لونچ ف a ئ l arsss ssasssm oawnaumamamam
Create کا/کی launch file `robot_model_exercises/launch/exercise_system_launch.py`:
```python
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
            default_value='exercise_robot.یوآر ڈی ایف.xacro',
            description='یوآر ڈی ایف/XACRO description file کے ساتھ کا/کی روبوٹ'
        )
    )

    # Get یوآر ڈی ایف via xacro
    robot_description_content = کمانڈ(
        [
            PathJoinSubstitution([FindPackageShare("robot_model_exercises"), "یوآر ڈی ایف", "exercise_robot.یوآر ڈی ایف.xacro"])
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # روبوٹ State پبلشر (سے ROS2)
    robot_state_publisher_node = نود(
        پیکیج='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # ورک ایکس روبوٹ State پبلشر
    exercise_robot_state_publisher_node = نود(
        پیکیج='robot_model_exercises',
        executable='exercise_robot_state_publisher',
        name='exercise_robot_state_publisher',
        output='both'
    )

    # Sensor Transformer
    sensor_transformer_node = نود(
        پیکیج='robot_model_exercises',
        executable='sensor_transformer',
        name='sensor_transformer',
        output='both'
    )

    # TF Visualizer
    tf_visualizer_node = نود(
        پیکیج='robot_model_exercises',
        executable='tf_visualizer',
        name='tf_visualizer',
        output='both'
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            exercise_robot_state_publisher_node,
            sensor_transformer_node,
            tf_visualizer_node,
        ]
    )
```
## vr ک jais 6: s ی sasm ٹی sasun گ اوسروس کی کی

###

1.
```bash
cd ~/ros2_ws
colcon build --packages-select robot_model_exercises
source install/ترتیب.bash
```
2.
```bash
ros2 launch robot_model_exercises exercise_system_launch.py
```
3.
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Monitor TF transforms
ros2 run tf2_ros tf2_monitor

# Echo specific transforms
ros2 run tf2_ros tf2_echo base_link lidar_link
```
4. ** تتور ک r یں
```bash
ros2 run rviz2 rviz2
```
مِس روف:
- یک a یک یک یک rewbous mwaucl ڈ saul ے aaml ari یں
- گدا ٹی ٹی ٹی ڈ sal saul ے aml ک aml ک v v wwauchum ٹ maam ٹ ransasaurmi
- ش aml ک ri یں مورپریر ی ڈ sacl ے ک so id یکھیں // کی ٹی ٹی ٹی ٹی ٹی ٹی ٹی vaunlaiaun۔

## vr ک JAS 7: پ rauarmins آپٹی maiun awors ڈی Bachn گ

### آپٹی maa ئزڈ naق

i یک آپٹ aa ئزڈ vr ژ n bnaa ئیں کے SAAT ھ کیچ کیچ کیچ کیچ کیچ کیچ کیچ ک ک ک ک ک ک حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ
```python
# robot_model_exercises/robot_model_exercises/optimized_sensor_transformer.py
import rclpy
سے rclpy.نود import نود
سے sensor_msgs.msg import LaserScan
سے geometry_msgs.msg import PointStamped
سے tf2_ros import TransformListener, Buffer
سے tf2_ros import TransformException
سے rclpy.qos import QoSProfile
import time
سے collections import defaultdict

class OptimizedSensorTransformer:
    def __init__(self):
        super().__init__('optimized_sensor_transformer')
        
        # Create ایک transform buffer کے ساتھ cache
        self.tf_buffer = Buffer(
            cache_time=rclpy.duration.Duration(seconds=10.0))  # 10s cache
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create QoS profile
        qos = QoSProfile(depth=5)  # Reduce depth کے لیے performance
        
        # Subscribe کو laser scan
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos)
        
        # پبلشر کے لیے transformed data
        self.transformed_scan_pub = self.create_publisher(
            LaserScan, 'scan_in_base_link', qos)
        
        # Cache کے لیے transforms وہ don't change frequently
        self.transform_cache = {}
        self.cache_timeout = 0.1  # 100ms cache timeout
        
        # Statistics
        self.transform_count = 0
        self.cache_hits = 0
        
        # Timer کے لیے performance statistics
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('Optimized Sensor Transformer started')

    def scan_callback(self, msg):
        """Transform laser scan سے sensor frame کو base_link frame کے ساتھ optimization"""
        start_time = time.time()
        
        try:
            # Check cache پہلا
            cache_key = f"{msg.header.frame_id}_to_base_link"
            
            اگر cache_key میں self.transform_cache:
                cached_transform, timestamp = self.transform_cache[cache_key]
                current_time = self.get_clock().اب().nanoseconds / 1e9
                
                اگر current_time - timestamp < self.cache_timeout:
                    self.cache_hits += 1
                    transform = cached_transform
                else:
                    # Cache expired, get fresh transform
                    transform = self.tf_buffer.lookup_transform(
                        'base_link', msg.header.frame_id, msg.header.stamp,
                        timeout=rclpy.duration.Duration(seconds=0.5))
                    # Update cache
                    self.transform_cache[cache_key] = (transform, current_time)
            else:
                # نہیں cache entry, get fresh transform
                transform = self.tf_buffer.lookup_transform(
                    'base_link', msg.header.frame_id, msg.header.stamp,
                    timeout=rclpy.duration.Duration(seconds=0.5))
                # Add کو cache
                current_time = self.get_clock().اب().nanoseconds / 1e9
                self.transform_cache[cache_key] = (transform, current_time)
            
            # Transform کا/کی laser scan (simplified - میں real نفاذ, 
            # آپ'd transform each point)
            transformed_scan = msg
            transformed_scan.header.frame_id = 'base_link'
            self.transformed_scan_pub.publish(transformed_scan)
            
            self.transform_count += 1
            
            # Log performance info periodically
            اگر self.transform_count % 100 == 0:
                end_time = time.time()
                self.get_logger().info(
                    f'Transform performance: {(end_time - start_time)*1000:.2f}ms, '
                    f'Cache hit rate: {self.cache_hits/max(1, self.transform_count):.2%}')
        
        except TransformException کے طور پر ex:
            self.get_logger().warn
        except Exception کے طور پر e:
            self.get_logger().error

    def print_stats(self):
        """Print performance statistics"""
        cache_hit_rate = self.cache_hits / max(1, self.transform_count) اگر self.transform_count > 0 else 0
        self.get_logger().info(
            f'TF Performance Stats - Transforms: {self.transform_count}, '
            f'Cache Hits: {self.cache_hits}, '
            f'Cache Hit Rate: {cache_hit_rate:.2%}')

def main(args=None):
    rclpy.init(args=args)
    نود = OptimizedSensorTransformer()
    
    try:
        rclpy.spin
    except KeyboardInterrupt:
        نود.get_logger().info
    finally:
        نود.destroy_node()
        rclpy.shutdown()
```
## vr ک ias 8: موتری کہ مساؤل ک a aaaul ہ ک srna

### ڈی BAUN گ یsaal
Create ایک debugging script `robot_model_exercises/test/tf_debugger.py`:
```python
import rclpy
سے rclpy.نود import نود
سے tf2_ros import Buffer, TransformListener
سے std_msgs.msg import String
سے rclpy.qos import QoSProfile

class TFDebugger:
    def __init__(self):
        super().__init__('tf_debugger')
        
        # Create ایک transform buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # پبلشر کے لیے debug messages
        qos = QoSProfile(depth=10)
        self.debug_pub = self.create_publisher(String, 'tf_debug_output', qos)
        
        # Timer کو run debug checks
        self.timer = self.create_timer(2.0, self.run_debug_checks)
        
        self.get_logger().info('TF Debugger started')

    def run_debug_checks(self):
        """Run various TF-related diagnostic checks"""
        self.get_logger().info('--- TF Debug Report ---')
        
        # Check 1: List تمام available frames
        try:
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
            self.get_logger().info(f'Available frames:\n{frames_yaml}')
        except Exception کے طور پر e:
            self.get_logger().error(f'Error getting frames: {e}')
        
        # Check 2: Validate common transforms
        common_transforms = [
            ('base_link', 'base_footprint'),
            ('base_link', 'lidar_link'),
            ('base_link', 'camera_link'),
            ('odom', 'base_footprint'),
        ]
        
        کے لیے parent, child میں common_transforms:
            try:
                self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                self.get_logger().info
            except Exception کے طور پر e:
                self.get_logger().warn
        
        # Check 3: Transform latency
        try:
            # Get کا/کی زیادہ تر recent transform کو check timestamp
            transform = self.tf_buffer.lookup_transform('base_link', 'lidar_link', rclpy.time.Time())
            stamp = transform.header.stamp
            current_time = self.get_clock().اب()
            
            latency = (current_time.nanoseconds - rclpy.time.Time.from_msg(stamp).nanoseconds) / 1e9
            self.get_logger().info
        except Exception کے طور پر e:
            self.get_logger().warn
        
        # Check 4: TF tree connections
        try:
            # Try کو find path between common frames
            self.tf_buffer.lookup_transform('odom', 'lidar_link', rclpy.time.Time())
            self.get_logger().info('✓ Full path available: odom -> lidar_link')
        except Exception کے طور پر e:
            self.get_logger().warn
        
        self.get_logger().info

def main(args=None):
    rclpy.init(args=args)
    نود = TFDebugger()
    
    try:
        rclpy.spin
    except KeyboardInterrupt:
        نود.get_logger().info
    finally:
        نود.destroy_node()
        rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## خ LAA صہ

عمال اعمت وِسر ِ ِ ِ ڈھ ی ی ی ی ی ی ی ی ی ڈھ

1.
2.
3.
4.
5.
6.

آپ re کھ t ے ہیں ہیں کی کی s ے ے کی si:
- ی oaur ڈی ڈی ڈی ڈی/xacro ک asamal artai ہ oi ئے پیچی d ہ jd ہ rewbo maa ڈ l banaa
- اِنوومام ٹی ٹی یف 2 کے saaٹ ھ rewbo ٹ reausasat کی
- ک vaur ڈی n یٹ ف raumw ں کے mababaghn sunssr ڈیٹ a ک se ک s
- TF کی کارکردگی کو بہتر بنائیں
- عام TF درختوں کے مسائل کو ڈیبگ کریں

عمورت ہیں اعوروری کے l یے ک ک o ک تو جور جولائی پیچی پیچی ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ

یہ ک ٹی ٹی ٹی ٹی ٹی ٹی پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ ٹی ٹی پ ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی پ پ ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی پ ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ک پ ک پ پ
