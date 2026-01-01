---


sidebar_position: 2
difficulty: beginner


---
# 4.2: تحم یsowr ڈی ڈی ِ ِ (مت ح ح ح ح ح ر شک شک

## ج a ج

الل لِل ، ماماوول نِنحن اوسر ڈی ا (اوسوون بوبو کی تال کی شک شک شک شک شک شک ک ک ک ک ک ک ف ف ف ف ف ف ف ف ف کی/کی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی کیی ڈ کیی ڈ ڈ کیی کیی ڈ ڈ ڈ کیی کیی کیی کیی کیی کیی کیی ڈ ڈ ڈ ڈ ڈ ، ، ، ،

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- asa/کی ڈھ ڈھ ک ک ک ک ک ک ک ، ، ، ، ، ، ، ، ، ، ، ، ، ،
- vausr ڈی ڈی ڈی ڈی ڈی ڈی کے کے کے کے کے کے کے li saadہ ہmarے ہ پیچی پیچی پیچی پیچی ہ ہ ہ
- ln ک s ، ج ج oi ڑ ، ، ، raiaauri ٹیز کی wauaaua ح ک ri یں
- isam کے SAAT ھ BACHR ی ، TAUADM ، ، ، ، ، ، ، ،
- ruboce_sٹیٹ_پblیشr ک a atamal ک comumos ss ے ی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی
- Debug common یوآر ڈی ایف issues

## vausr ڈی ڈی ڈی ڈی ڈی bin ی adatatatatat

### کی a ہے ہے ی ی ی vaur ڈی ڈی؟

ی اواور ڈی ڈی ڈی ڈی ڈی ڈی یہ waua ح at:

۔
- ** vus
- ** بصری خصوصیات
- ** تصادم کی خصوصیات
- ** جڑواں خصوصیات
- ** موواد **: شnau کی saa خ t کے ll یے tar

### vausr ڈی a یف ڈھ ڈھ ڈھ
```xml
<?xml version="1.0"?>
<روبوٹ name="my_robot">
  <!-- Define materials -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  
  <!-- Define links -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  
  <!-- Define joints -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.3 -0.1" rpy="0 0 0"/>
  </joint>
  
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </visual>
  </link>
</روبوٹ>
```
## لن ک تال

### لنکس اعد

یک لنک ایم آئی ایس او ایسور ڈی ا یف ا یف یک یک یک saut ج Sam کی sam کی Sam کی inmaaiound گی ک ک ک ہے۔ ہے۔ ہے۔

1.
2.
3.

### bin ی aad ی ln ک maamal
```xml
<link name="base_link">
  <!-- Visual properties -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Box کے ساتھ dimensions 0.5m x 0.3m x 0.1m -->
      <box size="0.5 0.3 0.1"/>
    </geometry>
    <material name="light_grey"/>
  </visual>
  
  <!-- Collision properties -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.5 0.3 0.1"/>
    </geometry>
  </collision>
  
  <!-- Inertial properties -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>
```
### vumaur ی کی کی sasam

اوسر ڈی ڈی کئی کئی جی jumaur ی sosaam کی maauthat ک irata ہے:
- **Box**: `<box size="x y z"/>`
- **Cylinder**: `<cylinder radius="r" length="l"/>`
- **Sphere**: `<sphere radius="r"/>`
- **Mesh**: `<mesh filename="پیکیج://path/کو/mesh.stl" scale="x y z"/>`
## ماؤٹر کہ ٹیزر یف

### ماؤتری کہ سمام

ی OAR ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ح ح ح ح ح ح ح کی ح ح Maauta ک Maauta ہے:

1.
2
3.
4
5
6

### ماؤتری کہ maual یں
```xml
<!-- Fixed joint -->
<joint name="sensor_mount" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Revolute joint -->
<joint name="arm_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Rotation axis -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>  <!-- Limits -->
</joint>

<!-- Continuous joint (free rotation) -->
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0 0.2 -0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Wheel rotation axis -->
</joint>

<!-- Prismatic joint (linear motion) -->
<joint name="slider_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slider_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.5" effort="100" velocity="0.5"/>
</joint>
```
## موریکول ز اوارس رنگ

### مووواد کی تعریف
```xml
<material name="red">
  <color rgba="0.8 0.0 0.0 1.0"/>
</material>

<material name="green">
  <color rgba="0.0 0.8 0.0 1.0"/>
</material>

<material name="blue">
  <color rgba="0.0 0.0 0.8 1.0"/>
</material>

<material name="white">
  <color rgba="1.0 1.0 1.0 1.0"/>
</material>

<material name="black">
  <color rgba="0.0 0.0 0.0 1.0"/>
</material>
```
## میمومل ط vr پ rir rrobobi ی maamal

### -saad ہ تور یق ڈ raauso rrwbo
```xml
<?xml version="1.0"?>
<روبوٹ name="diff_drive_robot">
  <!-- Materials -->
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
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>
  
  <!-- بائیں wheel -->
  <link name="left_wheel_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="white"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>
  
  <!-- صحیح wheel -->
  <link name="right_wheel_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="white"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>
  
  <!-- IMU sensor -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  
  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0 0.2 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <origin xyz="0 -0.2 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>
</روبوٹ>
```
## vausr ڈی ڈی کے کے saat ھ xacro

### تعار x xacro

xacro ہے ia یک mamrw ز bain کے li ِ s پی da ک da ک dai vali ی oaur ڈی ai ف aaul یں۔ یہ اج ج ز ز ہے:

- Parameterization کا یوآر ڈی ایف models
- مامو اوتر یفیں
- رع ق کے کے tttttttttttttttata
- ف a ئ l ش aaml -shrna

### بن ی AAD ی JRW maamal
```xml
<?xml version="1.0"?>
<روبوٹ xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">
  
  <!-- Define properties -->
  <xacro:property name="base_width" value="0.3"/>
  <xacro:property name="base_length" value="0.4"/>
  <xacro:property name="base_height" value="0.1"/>
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_width" value="0.04"/>
  <xacro:property name="PI" value="3.1415926535897931"/>
  
  <!-- Define ایک macro کے لیے wheels -->
  <xacro:macro name="wheel" params="prefix reflect">
    <link name="${prefix}_wheel_link">
      <visual>
        <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
      
      <collision>
        <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      
      <inertial>
        <mass value="0.2"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>
    
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel_link"/>
      <origin xyz="0 ${reflect * base_width/2 - wheel_width/2} -${base_height/2}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>
  
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </visual>
    
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.8"/>
    </inertial>
  </link>
  
  <!-- Create wheels using macro -->
  <xacro:wheel prefix="بائیں" reflect="1"/>
  <xacro:wheel prefix="صحیح" reflect="-1"/>
  
</روبوٹ>
```
## روبو ، re ی asat پ bl ش r

###
کا/کی `robot_state_publisher` نود takes ایک یوآر ڈی ایف اور joint positions کو publish کا/کی resulting transforms کو TF:
```bash
# Launch روبوٹ state پبلشر
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="path_to_urdf"
```
### nwad n ف aa ذ maual
```python
import rclpy
سے rclpy.نود import نود
سے sensor_msgs.msg import JointState
سے tf2_ros import TransformBroadcaster
سے geometry_msgs.msg import TransformStamped
import math

class JointStatePublisher:
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Create ایک پبلشر کے لیے joint states
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Create ایک transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer کو publish states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        self.time = 0.0

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        
        # Simulate oscillating joint positions
        self.time += 0.1
        left_pos = math.sin(self.time) * 0.5
        right_pos = math.cos(self.time) * 0.5
        
        msg.position = [left_pos, right_pos]
        msg.header.stamp = self.get_clock().اب().to_msg()
        msg.header.frame_id = 'joint_states'
        
        # Publish کا/کی message
        self.joint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    نود = JointStatePublisher()
    rclpy.spin
    نود.destroy_node()
    rclpy.shutdown()
```
## ک aam urnaa کے Saaat ی Maus ف aaul یں

### 3d ما a ی sautat

ی vaur ڈی ڈی ڈی ڈی ڈی ک ک ک ک ک ک ہے ہے ہے ح ح vaul ہ Bachroni ی 3d Maa ڈ L (stl ، dae ، Wa غی r ہ):
```xml
<link name="gripper_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Reference کو ایک STL file میں کا/کی mesh پیکیج -->
      <mesh filename="پیکیج://my_robot_description/meshes/gripper.stl" scale="1 1 1"/>
    </geometry>
    <material name="light_grey"/>
  </visual>
  
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="پیکیج://my_robot_description/meshes/gripper.stl" scale="1 1 1"/>
    </geometry>
  </collision>
</link>
```
### وولا (DAE) ف A ئ L یں بونانا

ll ll یے Maud پیچی پیچی پیچی پیچی پیچی پیچی کے کے کے کے bunauc:
```xml
<visual>
  <geometry>
    <mesh filename="پیکیج://my_robot_description/meshes/complex_model.dae"/>
  </geometry>
</visual>
```
## vausr ڈی ڈی یف یف ٹ ٹ Oli ٹ ٹ کی کی کی کی کی کی t tttatt

### vausr ڈی ڈی یف یف یف
```bash
# Check یوآر ڈی ایف validity
check_urdf /path/کو/روبوٹ.یوآر ڈی ایف

# یا use xacro کو check:
ros2 run xacro xacro -o output.یوآر ڈی ایف input.یوآر ڈی ایف.xacro
```
### ی vaur ڈی ڈی ڈی ڈی ی
```bash
# View کا/کی روبوٹ model
ros2 run rviz2 rviz2

# میں RViz, add ایک RobotModel display اور set کا/کی روبوٹ description پیرامیٹر
# Alternatively, use کا/کی کمانڈ line viewer:
ros2 launch urdf_tutorial display.launch.py model:=path/کو/روبوٹ.یوآر ڈی ایف
```
## روبو تال لونچ

###
```python
# launch/robot_description.launch.py
سے launch import LaunchDescription
سے launch.substitutions import کمانڈ, PathJoinSubstitution
سے launch_ros.ایکشنز import نود
سے launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get یوآر ڈی ایف via xacro
    robot_description_content = کمانڈ(
        [
            PathJoinSubstitution([FindPackageShare("my_robot_description"), "یوآر ڈی ایف", "روبوٹ.یوآر ڈی ایف.xacro"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = نود(
        پیکیج='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([
        node_robot_state_publisher
    ])
```
## عام اومور پ پ ی ی ی ی ی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ح ح

### 1. Mamamtr ح ح ح ح ح ح ح ح ح ح ح ح ح ح ح ح ح کہ ح ح .. ج 1 1 1.
```xml
<!-- Problem: Revolute joint without limits -->
<joint name="problematic_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <axis xyz="0 0 1"/>
  <!-- Missing limit tag! -->
</joint>

<!-- Solution: Add proper limits -->
<joint name="fixed_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>
```
### 2
```xml
<!-- Problem: Zero mass یا inertia -->
<link name="problem_link">
  <inertial>
    <mass value="0"/>  <!-- Zero mass ہے invalid -->
    <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>  <!-- Zero inertia ہے invalid -->
  </inertial>
</link>

<!-- Solution: Use reasonable values -->
<link name="fixed_link">
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```
### 3.
یوآر ڈی ایف چاہیے رکھتے ہیں ایک single root link (typically `base_link`):
```xml
<!-- Make sure آپ رکھتے ہیں ایک root link کے ساتھ نہیں parent -->
<link name="base_link">
  <!-- Definition -->
</link>
<!-- تمام other links connect کو یہ یا descendants کا یہ -->
```
### 4
```xml
<!-- Problem: Disconnected links میں کا/کی kinematic chain -->
<!-- Solution: تمام links ضرور ہونا connected through joints -->
```
## اعل ی ی کی ی ی vaur ڈی ڈی خص

### گیز BW- mauso ٹیگز

کے لِل سوسمولن مِس بوبو:
```xml
<link name="sensor_link">
  <visual>
    <!-- Visual properties -->
  </visual>
  
  <!-- گیزبو-specific properties -->
  <گیزبو reference="sensor_link">
    <material>گیزبو/Blue</material>
    <turnGravityOff>false</turnGravityOff>
  </گیزبو>
</link>

<گیزبو>
  <!-- گیزبو plugin کے لیے sensors -->
  <plugin name="sensor_plugin" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>imu</namespace>
      <remapping>~/out:=imu_data</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
</گیزبو>
```
## vausr ڈی ڈی ڈی ڈی bahatriی ی n ط ri یقہ ک ar
1. **Use consistent naming**: Follow ROS conventions (`base_link`, `base_footprint`, etc.)
2.
3. **Validate regularly**: Check آپ کا یوآر ڈی ایف کے ساتھ `check_urdf` کمانڈ
4.
5.
6.

## خ LAA صہ

یہ الل l ی ں

- ڈھ ڈھ ڈھ ڈھ چہ چہ ی ی aur ی ی ڈی ڈی ڈی ڈی ف ف ف ف ف ف ف ف ف ف ف ف ف ف
- ln ک s ، ج ج oi ڑ ، ، اواور اواون الالعل ٹیز
- کام احرنا کے ssaٹ ھ bachlrی ، taaldm ، ، ، اوسروس inertial uswautat
- xacro کے l یے l یے l یے پی ramaura ئزڈ rewbo کی wauaa ح t ک a ک a sastaamail
- روبوبو_س ٹیٹ_بلییشر ک a ک asatahamaul ک Chorati vi ئے rewbobo a کہ کہ کہ ہے
- مشترکہ مسائل اوار بِلرین اوریئر

ی کے کے کے کے کے ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک کے کے کے کے کے ک کے ک ک ک کے کے کے کے کے کے ک ک ک ک کے کے کے ہ ہ ہ ک ک ک ہ کے کے ہ ہ ہ ہ ہ ہ کے کے کے کے ہ ہ ہ ہ ہ کے کے کے ہ ہ ہ ہ ہ کے کے کے کے ہ ہ ہ ہ کے کے کے کے کے ہ ہ ہ ہ کے کے کے ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ک ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ am isri ک ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ک ہ ک ک
