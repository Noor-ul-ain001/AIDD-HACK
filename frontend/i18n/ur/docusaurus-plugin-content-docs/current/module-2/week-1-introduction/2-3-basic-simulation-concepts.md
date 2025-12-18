---


sidebar_position: 3
difficulty: intermediate


---
# 2.3: بنییہ سوزولہن تحقورات اواورس ورلڈ بلڈنی

## ج a ج

یہ الل l ی ہ m ک a/کی sdf (smwlixin tatail کی شک l) tla a ک ک r یں

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- smaun ے ک a/کی sdf (smwlixin tatal فصی شک شک شک شک ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ
- اوارس کو اوسن ی کے کے کے کے mauab ق Buna ئیں bunai گیز bw verli کے SAAT ھ maauli آ آ ​​آ آ
- Spawn اور control robots میں سمولیشن
- کam کے SAAT ھ bun ی ad ی suncer کے nِnli کے Jauli maawal
- سمولین پیرامیٹرز کو سمجھیں
- لون چ ف a ئ l یں bna ئیں - خ ud ک ar ط r یقے ط r یقے ssusuln wr ک lwaus بونا ئیں

## SDF (smwlaisn tatal شک شک l) ج a ئزہ

### کی A ہے SDF؟

یs ڈی ڈی ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ نیچے ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ح ح

- دنیا (ماؤل)
۔
- babai کی کی ttttt tt tt tt tt tt at
- لالیس
- پ لِس اون

### bin ی aad ی sdf ڈھ an چہ
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="example_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <model name="my_robot">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="chassis">
        <pose>0 0 0.1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.5 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.5 0.2</size>
            </box>
          </geometry>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>0.4</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.25</iyy>
            <iyz>0</iyz>
            <izz>1.25</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```
## اعنیی ی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی شکی کی کی شکی شکی کی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی شکی کی کے کے کے کے کے کے کے کی کی کی کی J کے کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی

### bin ی aad ی idn ی a ڈھ a ڈھ an چہ
Let's build ایک simple world file step کے ذریعے step. Create `my_robot_gazebo/worlds/basic_world.sdf`:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_world">
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Configure physics -->
    <physics name="ode" default="0" type="ode">
      <gravity>0 0 -9.8</gravity>
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
    
    <!-- Add ایک simple wall -->
    <model name="wall">
      <static>true</static>
      <pose>2 0 1 0 0 0</pose>
      <link name="wall_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 4.0 2.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 4.0 2.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Add another wall perpendicular کو کا/کی پہلا -->
    <model name="wall2">
      <static>true</static>
      <pose>-1 2 1 0 0 1.57</pose>
      <link name="wall2_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 4.0 2.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 4.0 2.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Add ایک box وہ کر سکتا ہے ہونا moved -->
    <model name="movable_box">
      <pose>-1 0 0.5 0 0 0</pose>
      <link name="box_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 1.0 1</ambient>
            <diffuse>0.5 0.5 1.0 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.083</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.083</iyy>
            <iyz>0</iyz>
            <izz>0.083</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```
### اعلی دِل کی دنیا کی اوست
Create ایک مزید advanced world file پر `my_robot_gazebo/worlds/advanced_world.sdf`:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="advanced_world">
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Configure physics کے ساتھ مزید detailed parameters -->
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
    
    <!-- Define plugins کے لیے ROS 2 integration -->
    <plugin name="gazebo_ros_init" filename="libgazebo_ros_init.so">
      <ros>
        <namespace>/گیزبو</namespace>
      </ros>
    </plugin>
    
    <!-- Add ایک custom light -->
    <light name="custom_light" type="point">
      <pose>3 3 8 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <attenuation>
        <range>10</range>
        <constant>0.5</constant>
        <linear>0.1</linear>
        <quadratic>0.01</quadratic>
      </attenuation>
      <cast_shadows>true</cast_shadows>
    </light>
    
    <!-- Add ایک cuboid obstacle -->
    <model name="obstacle_1">
      <static>false</static>
      <pose>1 1 0.2 0 0 0</pose>
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
            <ambient>1.0 0.0 0.0 1</ambient>
            <diffuse>1.0 0.0 0.0 1</diffuse>
            <specular>0.8 0.2 0.2 1</specular>
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
    
    <!-- Add ایک cylinder obstacle -->
    <model name="cylinder_obstacle">
      <static>false</static>
      <pose>-1 -1 0.5 0 0 0</pose>
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
            <ambient>0.0 1.0 0.0 1</ambient>
            <diffuse>0.0 1.0 0.0 1</diffuse>
            <specular>0.2 0.8 0.2 1</specular>
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
    
    <!-- Add ایک simple ramp -->
    <model name="ramp">
      <static>true</static>
      <pose>2 -2 0 0 0.3 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>file://meshes/ramp.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file://meshes/ramp.dae</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```
## روبو مامال الانمام

### آ سان روبو مامال الالالہ
Create `my_robot_gazebo/models/simple_robot/model.sdf`:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_robot">
    <pose>0 0 0.5 0 0 0</pose>
    
    <!-- Chassis -->
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.0 0.8 0.8 1</ambient>
          <diffuse>0.0 0.8 0.8 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
      <inertial>
        <mass>5</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.2</iyy>
          <iyz>0</iyz>
          <izz>0.25</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- بائیں wheel -->
    <link name="left_wheel">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.002</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- صحیح wheel -->
    <link name="right_wheel">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.002</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- Joints connecting wheels کو chassis -->
    <joint name="left_wheel_joint" type="continuous">
      <parent>chassis</parent>
      <child>left_wheel</child>
      <pose>0.15 0.2 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>
    
    <joint name="right_wheel_joint" type="continuous">
      <parent>chassis</parent>
      <child>right_wheel</child>
      <pose>0.15 -0.2 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>
  </model>
</sdf>
```
### maa ڈ l atail ف aa ئ l
Create `my_robot_gazebo/models/simple_robot/model.config`:
```xml
<?xml version="1.0"?>
<model>
  <name>Simple روبوٹ</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  
  <author>
    <name>آپ کا Name</name>
    <email>آپ کا.email@مثال.com</email>
  </author>
  
  <description>
    ایک simple روبوٹ model کے لیے سمولیشن.
  </description>
</model>
```
## روبوسس کو پ روورامام کے ماؤبی پھی لانا

### ros 2 nwau کے l یے l یے rewbous spawning
Create `my_robot_gazebo/my_robot_gazebo/spawn_robot.py`:
```python
#!/usr/bin/env python3
import rclpy
سے rclpy.نود import نود
سے gazebo_msgs.srv import SpawnEntity
سے geometry_msgs.msg import Pose
import os
سے ament_index_python.packages import get_package_share_directory

class RobotSpawner:
    def __init__(self):
        super().__init__('robot_spawner')
        
        # Create ایک سروس client کے لیے spawning entities
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Wait کے لیے کا/کی سروس کو ہونا available
        جب تک نہیں self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info
        
        self.spawn_robot()

    def spawn_robot(self):
        # Load کا/کی روبوٹ model سے file
        sdf_file_path = os.path.join(
            get_package_share_directory('my_robot_gazebo'),
            'models', 'simple_robot', 'model.sdf'
        )
        
        کے ساتھ کھلا(sdf_file_path, 'r') کے طور پر sdf_file:
            robot_model_xml = sdf_file.read()
        
        # Create کا/کی request
        request = SpawnEntity.Request()
        request.name = "my_robot"
        request.xml = robot_model_xml
        request.robot_namespace = ""
        
        # Set initial pose
        initial_pose = Pose()
        initial_pose.position.x = 0.0
        initial_pose.position.y = 0.0
        initial_pose.position.z = 0.5
        initial_pose.orientation.x = 0.0
        initial_pose.orientation.y = 0.0
        initial_pose.orientation.z = 0.0
        initial_pose.orientation.w = 1.0
        request.initial_pose = initial_pose
        
        # Call کا/کی سروس
        future = self.spawn_cli.call_async(request)
        self.get_logger().info
        
        # Wait کے لیے response
        rclpy.spin_until_future_complete(self, future)
        
        اگر future.result() ہے نہیں None:
            response = future.result()
            اگر response.success:
                self.get_logger().info
            else:
                self.get_logger().error
        else:
            self.get_logger().error

def main(args=None):
    rclpy.init(args=args)
    robot_spawner = RobotSpawner()
    
    try:
        rclpy.spin(robot_spawner)
    except KeyboardInterrupt:
        robot_spawner.get_logger().info
    finally:
        robot_spawner.destroy_node()
        rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## ک aam ک srn ے ے ے کے sast ی saunsr maus گیز bw

###
Create `my_robot_gazebo/models/sensor_robot/model.sdf` کے ساتھ sensors:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="sensor_robot">
    <pose>0 0 0.5 0 0 0</pose>
    
    <!-- Chassis کے ساتھ sensors -->
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.0 0.8 0.8 1</ambient>
          <diffuse>0.0 0.8 0.8 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
      <inertial>
        <mass>5</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.2</iyy>
          <iyz>0</iyz>
          <izz>0.25</izz>
        </inertia>
      </inertial>
      
      <!-- Camera sensor -->
      <sensor name="camera" type="camera">
        <pose>0.2 0 0.1 0 0 0</pose>
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
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
      </sensor>
      
      <!-- Laser scanner -->
      <sensor name="laser" type="ray">
        <pose>0.2 0 0.1 0 0 0</pose>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-1.57</min_angle>
              <max_angle>1.57</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.1</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <visualize>true</visualize>
      </sensor>
      
      <!-- IMU sensor -->
      <sensor name="imu_sensor" type="imu">
        <pose>0 0 0.1 0 0 0</pose>
        <always_on>1</always_on>
        <update_rate>50</update_rate>
        <visualize>false</visualize>
      </sensor>
    </link>
    
    <!-- بائیں wheel -->
    <link name="left_wheel">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.002</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- صحیح wheel -->
    <link name="right_wheel">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.002</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- Joints -->
    <joint name="left_wheel_joint" type="continuous">
      <parent>chassis</parent>
      <child>left_wheel</child>
      <pose>0.15 0.2 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>
    
    <joint name="right_wheel_joint" type="continuous">
      <parent>chassis</parent>
      <child>right_wheel</child>
      <pose>0.15 -0.2 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>
  </model>
</sdf>
```
## ف aa ئ l یں llan چ ک r یں کے ll ِ l ssowmolne

### ba ی s ک smwln lan چ ف aa ئ l
Create `my_robot_gazebo/launch/basic_simulation.launch.py`:
```python
import os
سے launch import LaunchDescription
سے launch.ایکشنز import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
سے launch.launch_description_sources import PythonLaunchDescriptionSource
سے launch.substitutions import LaunchConfiguration, PathJoinSubstitution
سے launch_ros.ایکشنز import نود
سے launch_ros.substitutions import FindPackageShare
سے launch.event_handlers import OnProcessExit
سے ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get کا/کی پیکیج directory
    pkg_share = get_package_share_directory('my_robot_gazebo')
    
    # World file path
    world_file = os.path.join(pkg_share, 'worlds', 'basic_world.sdf')
    
    # Launch گیزبو کے ساتھ کا/کی world
    گیزبو = ExecuteProcess(
        cmd=['گیزبو', '--verbose', world_file, 
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # روبوٹ State پبلشر کے لیے یوآر ڈی ایف
    urdf_file = os.path.join
    robot_state_publisher = نود(
        پیکیج='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Static transform پبلشر کے لیے base footprint
    static_tf_publisher = نود(
        پیکیج='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    # Joint State پبلشر
    joint_state_publisher = نود(
        پیکیج='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        remappings=[('/robot_description', '/robot_description')]
    )
    
    return LaunchDescription([
        گیزبو,
        robot_state_publisher,
        static_tf_publisher,
        joint_state_publisher,
    ])
```
### اعل ی کی ف ف ف ف ف ف ف آف آف آف آف آف آف آف آف آف آف آف ف ف ف ف ف ف ف ف ف ف ف آف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف کی ف ف ف ف ف ف ف ف ف ف کی ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف کی آف آف آف آف آف آف آف آف آف آف آف آف آف ؛)
Create `my_robot_gazebo/launch/advanced_simulation.launch.py`:
```python
import os
سے launch import LaunchDescription
سے launch.ایکشنز import IncludeLaunchDescription, ExecuteProcess
سے launch.launch_description_sources import PythonLaunchDescriptionSource
سے launch.substitutions import LaunchConfiguration, PathJoinSubstitution
سے launch_ros.ایکشنز import نود
سے launch_ros.substitutions import FindPackageShare
سے ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get کا/کی پیکیج directory
    pkg_share = get_package_share_directory('my_robot_gazebo')
    
    # Advanced world file path
    world_file = os.path.join(pkg_share, 'worlds', 'advanced_world.sdf')
    
    # Launch گیزبو کے ساتھ کا/کی world
    گیزبو = ExecuteProcess(
        cmd=['گیزبو', '--verbose', world_file, 
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # روبوٹ State پبلشر کے لیے یوآر ڈی ایف
    robot_state_publisher = نود(
        پیکیج='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Launch controller manager
    controller_manager = نود(
        پیکیج='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(pkg_share, 'config', 'my_robot_controllers.yaml'),
            {'use_sim_time': True}
        ],
        output='screen'
    )
    
    # Spawn controllers
    spawn_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    spawn_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'diff_drive_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        گیزبو,
        robot_state_publisher,
        controller_manager,
        spawn_joint_state_controller,
        spawn_velocity_controller,
    ])
```
## طbیsaesahat تال

### ط b ی aa ی at کے پی پی پی پی پی پی پی پی پی ک ک v ssmauna

اعاب ی ت گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز ے ے ے ے ے کی کی کی کی کی کی کی کی
```xml
<physics name="ode" default="0" type="ode">
  <gravity>0 0 -9.8</gravity>
  
  <!-- Solver settings -->
  <ode>
    <solver>
      <type>quick</type>  <!-- quick یا world -->
      <iters>100</iters>  <!-- Number کا iterations کے لیے error correction -->
      <sor>1.3</sor>      <!-- Successive کے اوپر Relaxation پیرامیٹر -->
    </solver>
    
    <!-- Constraint settings -->
    <constraints>
      <cfm>0.0</cfm>  <!-- Constraint Force Mixing -->
      <erp>0.2</erp>  <!-- Error Reduction پیرامیٹر -->
      <!-- Max velocity کے لیے contact correction -->
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <!-- Contact layer depth -->
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
  
  <!-- Time stepping settings -->
  <max_step_size>0.001</max_step_size>        <!-- سمولیشن time step -->
  <real_time_factor>1</real_time_factor>      <!-- سمولیشن speed relative کو real time -->
  <real_time_update_rate>1000</real_time_update_rate>  <!-- Update rate میں Hz -->
</physics>
```
### ک اراورڈ گی بومبل ہ ارسات گی طیارت

.
- **مزید solver iterations**: Higher accuracy لیکن slower سمولیشن
.
۔

## خ LAA صہ

یہ ا ص ص ص ص /////////////////////// ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص //////////////// ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص // ص ص ص ص ص ص ص ص // ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص/

- گدا ڈی ڈی ڈی ڈی ڈی (smwlaintan ttacl کی شک l) ina چہ اوسورس
- - اِن ی
- بل گ گ
۔
- کام احرنا کے ستی ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ
- لونچ ف a ئ la یں بونا ک ک ک ک ک آٹ omaus sumwln vr ک ف Clwi
- - سیب ی ی ی پ پ پ پ پ پ پ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ پ پ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ حفظ

ایس ایس ایم اے این ے یہ یہ ی ی jad jaidadad ہے ص کے کے کے کے کے کے ہی ہی ہی ہی ہی ہی ہی ہی ہی ہی ہی ہی ہی ہی ہی کے biad ، ، ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ
