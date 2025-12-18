---


sidebar_position: 3
difficulty: intermediate


---
# ستی 4: اوسوؤنسڈ سوسوولن

## ج a ج

یہ ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ج ج ج ج ی ی ی ی ی ی ی ی ی ی کھ ماما النک

## ss یکھ n ے کے maua ص d

کے ذ ک ک ک ک a/کی کی خ خ خ خ atatam ک a یہ یہ ہف ہف آپ آپ ک ک ک ک ک ک ک ک ے ے ے ے گ گ گ گ گ
- ملٹی روبوبو سومولان منظرناموں کو نافذ کریں
- اوسوسنس ڈ فزک s فزک s فزک ڈ aiaumaumauss maaulni گ ک a a ط laa ق ک ri یں
- سینسر فیوژن کو مربوط کریں
- سمولین کی کارکردگی کو بہتر بنائیں

## مول ٹی- روبوبو سوسولن

### متعادد روبوبو کو مربوط ہی

متعاد روبوبو کی نِن الکحل کے لِل ئے مِمتاستاد کی
```python
# مثال: Multi-روبوٹ controller
import rclpy
سے rclpy.نود import نود
سے geometry_msgs.msg import Twist

class MultiRobotController:
    def __init__(self):
        super().__init__('multi_robot_controller')
        
        # روبوٹ configurations
        self.robots = ['robot1', 'robot2', 'robot3']
        
        # Create publishers کے لیے each روبوٹ
        self.cmd_vel_pubs = {}
        کے لیے روبوٹ میں self.robots:
            topic_name = f'/{روبوٹ}/cmd_vel'
            self.cmd_vel_pubs[روبوٹ] = self.create_publisher(
                Twist,
                topic_name,
                10
            )
    
    def send_command(self, robot_name, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pubs[robot_name].publish(msg)
```
### llonچ ف aa ئ l کے ll mli rewbobsssmuln
```xml
<launch>
  <!-- Load روبوٹ descriptions -->
  <group>
    <push-ros-namespace namespace="robot1"/>
    <include file="$(find-pkg-share my_robot_description)/launch/spawn.launch.py">
      <arg name="robot_name" value="robot1"/>
      <arg name="x" value="0.0"/>
      <arg name="y" value="0.0"/>
    </include>
  </group>
  
  <group>
    <push-ros-namespace namespace="robot2"/>
    <include file="$(find-pkg-share my_robot_description)/launch/spawn.launch.py">
      <arg name="robot_name" value="robot2"/>
      <arg name="x" value="2.0"/>
      <arg name="y" value="0.0"/>
    </include>
  </group>
  
  <!-- Add مزید robots کے طور پر needed -->
</launch>
```
### tf ٹ ri maunaumn ٹ

ملچرو بوب ی ni ظ aamwa ں mautahaus انتھم کی ض ض ض ض ض ہ ہ ہ ی ی ی ی ی
```
map
├── robot1/odom
│   └── robot1/base_link
│       ├── robot1/laser
│       └── robot1/camera
└── robot2/odom
    └── robot2/base_link
        ├── robot2/laser
        └── robot2/camera
```
## ج ج ج ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط کی کی کی ط ج ج ج ج ج ج کی کی کی ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی ط ط ث ث ہے

### حقیق حقیق حقیق snadan ہ ط Baua ی at کی خص خص oa صی at

ف aa ئ n ٹ ٹ son son فزک ss پ sraa پ r ٹیز-حقیقی کے کے کے ط ط ط ط ط ط ط ط ط ط ط ط ط ط
```xml
<!-- میں یوآر ڈی ایف/SDF کے لیے realistic physics -->
<link name="wheel_link">
  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" 
             iyy="0.001" iyz="0.0" izz="0.002"/>
  </inertial>
  
  <collision>
    <!-- Use realistic collision geometry -->
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
  </collision>
  
  <گیزبو>
    <material>گیزبو/Blue</material>
    <mu1>100.0</mu1>  <!-- Friction coefficient -->
    <mu2>100.0</mu2>  <!-- Secondary friction -->
    <kp>10000000.0</kp>  <!-- Contact stiffness -->
    <kd>100000.0</kd>    <!-- Contact damping -->
    <max_vel>100.0</max_vel>
    <min_depth>0.001</min_depth>
  </گیزبو>
</link>
```
### ک saum فزک sis پ sl گ ann

ک saum فزک s فزک s کے کے کے کے کے کے کے l یے Mauchui ط l یے maus ط ط ط ط ط
```cpp
#include <گیزبو/گیزبو.hh>
#include <گیزبو/physics/physics.hh>
#include <گیزبو/common/common.hh>

class CustomPhysicsPlugin : public گیزبو::WorldPlugin
{
public:
    void Load
    {
        یہ->world = _world;
        
        // Connect کو pre-update event
        یہ->updateConnection = گیزبو::event::Events::ConnectWorldUpdateBegin(
            std::bind);
    }

private:
    void OnUpdate()
    {
        // Implement custom physics behavior
    }

    گیزبو::physics::WorldPtr world;
    گیزبو::event::ConnectionPtr updateConnection;
};

GZ_REGISTER_WORLD_PLUGIN(CustomPhysicsPlugin)
```
## سنسر یswoun Maum Ssowmwln

### مسعدد سیسنسر کی اقسام ک ایک امتز (

at پ snadan ہ saunsr فی owaun munahnamwau کی jnahal ی ک r یں:
```python
import rclpy
سے rclpy.نود import نود
سے sensor_msgs.msg import LaserScan, Imu, NavSatFix
سے geometry_msgs.msg import PoseWithCovarianceStamped
سے tf2_ros import TransformListener
سے rclpy.qos import QoSProfile, ReliabilityPolicy

class SensorFusionNode:
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Set اوپر QoS profiles کے لیے reliable sensor data
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Subscribe کو multiple sensor types
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile)
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, qos_profile)
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, qos_profile)
        
        # پبلشر کے لیے fused pose estimate
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/fused_pose', qos_profile)
        
        # Initialize sensor fusion الگورتھم
        self.initialize_fusion_algorithm()
    
    def lidar_callback(self, msg):
        # Process LIDAR data کے لیے localization
        self.process_lidar_for_localization(msg)
    
    def imu_callback(self, msg):
        # Process IMU data کے لیے orientation
        self.process_imu_for_orientation(msg)
    
    def gps_callback(self, msg):
        # Process GPS data کے لیے global position
        self.process_gps_for_position(msg)
```
## ssmwl یش n پ rauarmns آپٹی maidns

### mwaur sswmwln t ک n یک

1.
2.
3.
4

### آپٹ a ئزڈ گیز bo bu atail
```bash
# گیزبو server launch کے ساتھ optimizations
gzserver --verbose \
  --physics=ode \
  --play-speed=1.0 \
  worlds/my_world.world
```
```xml
<!-- Physics engine optimization -->
<physics name="default_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```
## st حقیق Snadan ہ maawal maa ڈ ln گ

###

smwlain maausoul ی Bnaa ئیں
```xml
<!-- Complex world کے ساتھ dynamic elements -->
<world name="complex_factory">
  <!-- Static elements -->
  <include>
    <uri>model://large_industrial_building</uri>
  </include>
  
  <!-- Dynamic elements -->
  <actor name="pedestrian_1">
    <pose>5 0 0.05 0 0 0</pose>
    <skin>
      <filename>walk.dae</filename>
      <scale>1.0</scale>
    </skin>
    <animation name="walking">
      <filename>walk.dae</filename>
      <scale>1.0</scale>
      <interpolate_x>true</interpolate_x>
    </animation>
    <waypoints>
      <waypoint>
        <time>0</time>
        <pose>5 0 0.05 0 0 0</pose>
      </waypoint>
      <waypoint>
        <time>10</time>
        <pose>5 10 0.05 0 0 0</pose>
      </waypoint>
      <!-- Additional waypoints -->
    </waypoints>
  </actor>
</world>
```
### mswsm اوسور ما a ی ویل ی ی ی ی ی ث

maausoula ی at ی ح alat کی n ق aal ی ک r یں۔
```xml
<!-- Lighting effects -->
<light name="sun" type="directional">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>1000</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <direction>-0.3 0.3 -1</direction>
</light>

<!-- Atmospheric effects -->
<scene>
  <fog type="linear">
    <color>0.8 0.8 0.8</color>
    <density>0.1</density>
    <start>5</start>
    <اختتام>100</اختتام>
  </fog>
</scene>
```
## عمالہ وورک اِس

ہف ہف t ہ 's vr ک ک s sssos maus auswansi ڈ muli rewbobss ssmolain کی tacli یق arsna ش aml ہے:

1.
2
3. سوزولن کی ک ک ک ک ک ک v Ba ہ tr bunaa ئیں کے ll ی re یئ l ٹ aaum پ Sam پ r aml li li driadd
4

## خ LAA صہ

ہف ہف ہف ہف ہف ہف ج ج d ی d atr ی n smowl ی n t ک nauc کی کی کی کھ کھ کی۔ کی۔ کی۔ کی۔ کی۔ کی آپ '' '' 'نِن ملک ماواال 2 کے اوتم احر۔
