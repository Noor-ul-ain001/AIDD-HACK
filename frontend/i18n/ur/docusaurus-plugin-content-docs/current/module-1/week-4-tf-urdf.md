---


sidebar_position: 4
difficulty: beginner


---
# سٹن 4: tf owar vur ی ڈی ڈی ڈی ڈی ڈی ڈی ڈی

## ج a ج

ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف یں یں یں یں یں یں k k k k k k k k کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے nmaidund گی ک ک ک ک ک ک ہ v ، tf (transforms): tf (arannssaurimis) یہ ٹ vl ز ہیں ض rorur ی کے ll یے rewboc خی aal ، ni ی sowauchn ، اوسروس سلر۔

## ss یکھ n ے کے maua ص d

کے ذ ک ک ک ک a/کی کی خ خ خ خ atatam ک a یہ یہ ہف ہف آپ آپ ک ک ک ک ک ک ک ک ے ے ے ے گ گ گ گ گ
- سمو کہ
۔
- Create اور interpret یوآر ڈی ایف files
- روبوبو رعقعت کے پبلی الکرز کa Jasatahamal Acrیں
- ف راہوم جٹبڈ ی لا ی se ک v jna فذ ک r یں
-

## TF (aransaurmiu) Maaaa revs 2

tf (aranssaurmiu) ہے ai یک پیکیج پیکیج پیکیج پیکیج پیکیج پیکیج ک/کی ص ari ف ک ک ک ک ک j ک ک ک یک یک یک یک یک یک کی کی کی کی کی یہ ک ک ک ک ک ک ک ک ک ک ک ک ک ک N یٹ ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک یک یک یک کی کی کی کی یہ ک ک یک یک یک کی کی کی کی کی یہ ک یک یک یک کی کی کی کی کی کی ک یک یک یک یک کی کی کی کی

### vausr ڈی n یٹ ف raum

یک ک waur ڈی n یٹ ف raum ہے ہے ہے ہے ہے یک یک یک یک reference ssiqum waus sn پ owaun کی waucaaut کی کی ک کamn iaamumaumaumaumaumaumaumaumaumaumaumaumaumaumaumaumaumaumaumaumaumaumaumaumaumaumaus mamaus mamaus mamamau mamaus mamamiu mamaus mamamai ma mamamai mamaus sawaut کی waucaaut کی waucaaut کی waucaaut کی
- `map`: World-fixed frame
- `odom`: Odometry-based frame
- `base_link`: روبوٹ's base frame
- `camera_frame`: Camera's reference frame
- `tool0`: Tool اختتام-effector frame
### ٹی ا یف 2 تالورات

ٹی ایف 2 ہے ک ک ک a/کی دوسرا جنرشن ک a ک a/کی -ٹ ٹ ٹransisaurm llaiaubraیriی maus ros. یہ ف ف raaum ک rata ہے:

1.
2.
3
4

### bin ی Adad ی ٹی ٹی ٹی S 2 Esatamail
```python
import rclpy
سے rclpy.نود import نود
سے tf2_ros import TransformException
سے tf2_ros.buffer import Buffer
سے tf2_ros.transform_listener import TransformListener
سے geometry_msgs.msg import TransformStamped

class FrameListener:
    def __init__(self):
        super().__init__('frame_listener')
        
        # Create ایک buffer کو store transforms
        self.tf_buffer = Buffer()
        
        # Create ایک transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer کو periodically lookup transforms
        self.timer = self.create_timer(1.0, self.lookup_transform)

    def lookup_transform(self):
        try:
            # Look اوپر transform سے 'base_link' کو 'camera_link' پر current time
            t = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                rclpy.time.Time())
                
            self.get_logger().info(
                f'Transform: ({t.transform.translation.x}, '
                f'{t.transform.translation.y}, '
                f'{t.transform.translation.z})')
                
        except TransformException کے طور پر ex:
            self.get_logger().info
            return
```
### اعدت کی تبدالی
```python
import rclpy
سے rclpy.نود import نود
سے geometry_msgs.msg import TransformStamped
سے tf2_ros import TransformBroadcaster

class FramePublisher:
    def __init__(self):
        super().__init__('frame_publisher')

        # Create ایک transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer کو broadcast transforms
        self.timer = self.create_timer(0.1, self.broadcast_transform)

    def broadcast_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().اب().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'

        # Define کا/کی transform
        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast کا/کی transform
        self.tf_broadcaster.sendTransform(t)
```
## vausr ڈی ڈی ڈی ڈی

اوسر ڈی ا یک یک ہے xml ف armiu کے کے کے کے ے ے jharati ے jurati ہ swaus ہ sos erwaumaul کی jumaaund گی inmaaund گی ک ک ک ک ک ک ک ک ک ک ک ک یہ ک/کی کائینیٹک اورس متحرک ڈھانچے کی وضاحت کرتا ہے ، جس میں روابط ، جوڑ ، مواد ، اوار بصری/inertial خصوصیات شامل ہیں۔

### vausr ڈی ڈی ڈی ڈی ڈی

.
- ** vus
- **Visual**: کیسے کا/کی روبوٹ appears میں سمولیشن/visualization
- **Inertial**: Mass, مرکز کا mass, اور inertia properties
- ** تصادم **: تصادم کا پتہ لگانے جیومیٹری

### bin ی aad ی vausr ڈی ia یف maul
```xml
<?xml version="1.0"?>
<روبوٹ name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0"
               izz="0.1"/>
    </inertial>
  </link>

  <!-- Wheel links -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- Joints connecting wheels کو base -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="-0.15 0.25 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="-0.15 -0.25 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</روبوٹ>
```
## روبو ، re ی asat پ bl ش r

ک a/کی rewbwau_siٹیٹ_پblaیشr پیکیج ی ی/کی mautriکہ jaustriکہ زasawauch much ک a aiassrobw bochobochos ass asas asas asab ح ک ح ح ح ک ح ک ح ح ح ح ح ح ک ح ک ح ح ک ک ک ک ک ک ک ک ک ک ک ک ک ح ک ک ک ک ک ک ک ک ح ح ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ln ک s۔ یہ // کی کی کی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی 2 پیکیج ک ک ک ک ک ک ک ک ک ک ک ک ک ہ ہ ہ ہ ہ ہ کے کی کے کے کے کے کی کی کی کی کی کی کی کی

###
```python
import rclpy
سے rclpy.نود import نود
سے sensor_msgs.msg import JointState
سے std_msgs.msg import Header
سے tf2_ros import TransformBroadcaster
سے geometry_msgs.msg import TransformStamped
import math

class StatePublisher:
    def __init__(self):
        super().__init__('state_publisher')
        
        # Initialize joint state پبلشر
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer کو publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [math.sin(self.get_clock().اب().nanoseconds / 1e9),
                        math.cos(self.get_clock().اب().nanoseconds / 1e9)]
        msg.header.stamp = self.get_clock().اب().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Publish joint states
        self.joint_pub.publish(msg)
```
## tf awaur ی s ڈی ڈی یف ڈی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک

موڈ لامان-لائیون ٹ vli کے llulil ک aam ک ک ہیں ہیں sati ھ sati ھ tf awr avaur ڈی ڈی ڈی:
```bash
# TF tools
ros2 run tf2_tools view_frames                    # View TF tree
ros2 run tf2_ros tf2_echo frame_id                # Echo transform
ros2 run rviz2 rviz2                              # Visualize TF tree میں RViz

# یوآر ڈی ایف tools
check_urdf /path/کو/روبوٹ.یوآر ڈی ایف                    # Validate یوآر ڈی ایف
ros2 run xacro xacro /path/کو/روبوٹ.xacro > روبوٹ.یوآر ڈی ایف  # Process Xacro
```
## عمل ، موبا ئ ئ ڈی

یہ aa ں 's ia یک maud mauml Maul mubaul robubo ی oaur ڈی ia یف maual کے saati ھ tf atratiab:
```xml
<?xml version="1.0"?>
<روبوٹ name="mobile_robot" xmlns:xacro="http://ros.org/wiki/xacro">
  <material name="orange">
    <color rgba="1.0 0.5 0.0 1.0"/>
  </material>

  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0"
               iyy="0.083" iyz="0.0"
               izz="0.133"/>
    </inertial>
  </link>

  <!-- Camera link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- Camera joint -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- بائیں wheel -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.18 -0.1" rpy="-1.5708 0 0"/>
  </joint>

  <!-- صحیح wheel -->
  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0 -0.18 -0.1" rpy="-1.5708 0 0"/>
  </joint>
</روبوٹ>
```
## عام ٹی ٹی یف یف کے کے jasa ئ l avers ح ح

1.
2.
3
4.

## اوانضمام کے ساسا ھ ھ پrیsasiیکشn اوسن نِنونو

tf ہے ہے ہے کے کے کے l یے:
- سینسر فیوژن: سینسر ڈیٹا کو ٹرانسفارم
- لوکلائزیشن: نقشہ آور اوڈوم فریموں کے مابین تبدیل
- نانوسن: منڈ بب بنڈ کے کے کے کے کے کے کے کے کے کے کے کے
- ہی ra پھی r ی

## خ LAA صہ

ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف ی ی ی ی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی کے کے کے کے کے ، ، ، ، ، ، ض ض ض ض ض ض ت ض ت ض ض ض ض ض ض ض ض ض ض ض ض ض ، ، ، ، ، ، ، آ vr mautri re ی asastwau ک v ش aauad ک sr یں۔ یہ تالورات ہیں بن ی ad ی کے llul rewbo ، خی aal ، jniuchn ، ، ، ، ، ، ہی ہی ہی ہی

## ماؤس

1.
2. امل دل
3
4.
