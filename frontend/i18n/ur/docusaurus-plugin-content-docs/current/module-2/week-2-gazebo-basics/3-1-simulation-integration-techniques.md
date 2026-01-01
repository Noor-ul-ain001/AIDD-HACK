---


sidebar_position: 1
difficulty: beginner


---

# 3.1: روبوٹ سیمیولیشن کی ٹیکنیکس

## جائزہ

یہ ذیلی ماڈیول روبوٹ سیمیولیشن کی متعدد تکنیکوں اور ان کے ROS 2 کے ساتھ انضمام کو سمجھاتا ہے۔ ہم گیزبو اور دیگر سیمیولیشن ماحول کے ساتھ کام کریں گے۔

## سیکھنے کے مقاصد

اس ذیلی ماڈیول کے اختتام تک، آپ:

- Gazebo اور دیگر سیمیولیشن پلیٹ فارم کی بنیادیں سیکھیں گے
- ROS 2 کے ساتھ سیمیولیشن کے انضمام کے طریقے سیکھیں گے
- مختلف سیمیولیشن ٹیکنیکوں کا موازنہ کریں گے
- سیمیولیشن کے مسائل کو حل کرنا سیکھیں گے
- روبوٹ کی سیمیولیشن کو تشکیل دینا سیکھیں گے

## Gazebo: ایک مقبول سیمیولیشن پلیٹ فارم

### Gazebo کا تعارف

Gazebo ایک مقبول فزکس-محسوس سیمیولیشن سافٹ ویئر ہے جو ROS 2 کے ساتھ یکسگانہ طور پر کام کرتا ہے۔ یہ جسمانی دنیا کے مظاہر کو نقل کرتا ہے، بشمول:

- Fysics
- سینسر
- گرافیکل ماڈلنگ
- ورچوئل ماڈیولز

### Gazebo کی خصوصیات

1. ** فزکس انجن **: ODE، Bullet، یا DART کا استعمال کرتا ہے
2. ** سینسر ماڈلز **: لیزر، کیمرہ، IMU، اور دیگر سینسرز کے لیے ماڈلز
3. ** 3D رینڈر **: OpenGL کا استعمال کرتا ہے
4. ** پلگ ان سپورٹ **: کسٹم چیزوں کو شامل کرنے کے لیے
5. ** ROS 2 انضمام **: ros_gz_pkgs کے ذریعے

### Gazebo میں سیمیولیشن تشکیل دینا

Gazebo میں سیمیولیشن کو تشکیل دینے کے لیے ہم XML فائلز استعمال کرتے ہیں جسے SDF (Simulation Description Format) کہا جاتا ہے۔

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="small_room">
    <!-- Light -->
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
      <direction>-0.3 0.3 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0 0 0 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Cube object -->
    <model name="cube">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0.0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0.8 0 1</ambient>
            <diffuse>0 0.8 0 1</diffuse>
            <specular>0 0.2 0 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## ROS 2 اور Gazebo کا انضمام

### ros_gz_pkgs

ROS 2 اور Gazebo کے مابین انضمام ros_gz_pkgs کے ذریعے ہوتا ہے، جو درج ذیل میں شامل ہے:

- `ros_gz_bridge`: ROS 2 اور Gazebo پیغامات کے مابین ترجمہ
- `ros_gz_sim`: Gazebo سیمیولیشن کے لیے ROS 2 APIs
- `gz_ros2_control`: ROS 2 کنٹرول کا استعمال کرتے ہوئے Gazebo میں سینسر اور ایکٹویٹر

### Gazebo سیمیولیشن شروع کرنا

```bash
# Gazebo دنیا کو ros2 کے ساتھ چلانا
ros2 launch ros_gz_sim gazebo.launch.py world_name:=my_world.sdf
```

### مثال: ROS 2 نوڈ کے ذریعے Gazebo کو کنٹرول کرنا

```python
# Gazebo کو کنٹرول کرنے کا مثال کوڈ
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GazeboController(Node):
    def __init__(self):
        super().__init__('gazebo_controller')
        
        # /cmd_vel کو گیزبو میں شائع کریں
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/model/vehicle/cmd_vel', 
            10
        )
        
        # 0.1 سیکنڈ کے بعد پیغام بھیجیں
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = 1.0  # سامنے کی طرف حرکت
        msg.angular.z = 0.5  # گھومیں
        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    controller = GazeboController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## دیگر سیمیولیشن پلیٹ فارم

### Webots

Webots ایک مفت اور اوپن سورس روبوٹ سیمیولیشن سافٹ ویئر ہے:

```python
# Webots کے لیے مثال کوڈ
from controller import Robot, Motor, DistanceSensor

# Webots روبوٹ کی تنصیب
robot = Robot()

# ٹائم اسٹیپ
timestep = 64

# موٹر کی تنصیبات
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# سینسر کی تنصیبات
ds = robot.getDevice('distance sensor')
ds.enable(timestep)

# Webots میں ہر ایک چکر
while robot.step(timestep) != -1:
    # سینسر سے ڈیٹا حاصل کریں
    ds_value = ds.getValue()
    
    # کچھ کنٹرول کوڈ
    left_motor.setVelocity(1.0)
    right_motor.setVelocity(1.0)
```

### PyBullet

PyBullet سافٹ بอดی، فزکس، اور ڈیپ لرننگ کے لیے ٹولز فراہم کرتا ہے:

```python
# PyBullet کے لیے مثال کوڈ
import pybullet as p
import pybullet_data
import time

# PyBullet سرور کو لانچ کریں
physicsClient = p.connect(p.GUI)  # GUI یا DIRECT

# ڈیٹا پیتھ شامل کریں
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# فرش لوڈ کریں
p.loadURDF("plane.urdf")

# شیطان کو لوڈ کریں
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("r2d2.urdf", cubeStartPos, cubeStartOrientation)

# فزکس کی ترتیبات
p.setGravity(0, 0, -10)  # زمین کی گریویٹی

# سیمیولیشن لوپ
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)  # 240 Hz

# سرور کو منقطع کریں
p.disconnect()
```

## سیمیولیشن کی چیلنجز

### 1. حقیقت کا فرق

سیمیولیشن اور حقیقتی دنیا کے درمیان فرق:

- فزکس کے ماڈلز کی ناکافی مطابقت
- سینسر نوائز کا نقل ناکام
- کمپیوٹیشنل حدود
- ماحولیاتی متغیرات کا فقدان

### 2. کارکردگی کی چیلنج

- بڑے ماحول کی چلانے کی مشکل
- حقیقی وقت کی کارکردگی کے لیے سرور کی ضرورتیں
- متعدد روبوٹس کے لیے مسائل

### 3. ڈومین رینڈمائزیشن

حقیقت کے قریب ترین سیمیولیشن کے لیے ہم ڈومین رینڈمائزیشن کا استعمال کرتے ہیں:

```python
# ڈومین رینڈمائزیشن کی مثال
import random

class DomainRandomization:
    def __init__(self):
        self.friction_range = (0.1, 1.0)
        self.mass_range = (0.5, 2.0)
        self.lighting_range = (0.5, 1.5)
    
    def randomize_friction(self):
        return random.uniform(*self.friction_range)
    
    def randomize_mass(self):
        return random.uniform(*self.mass_range)
    
    def randomize_lighting(self):
        return random.uniform(*self.lighting_range)
    
    def apply_randomizations(self, model):
        # جسم کو نئے ویلو سے اپ ڈیٹ کریں
        new_friction = self.randomize_friction()
        new_mass = self.randomize_mass()
        # اور ڈیٹا کو ماڈل میں لاگو کریں
```

## سیمیولیشن کی تربیت (Sim-to-Real Transfer)

### ڈومین رینڈمائزیشن کا استعمال

سیمیولیشن سے حقیقت کے لیے اچھا ٹرانسفر حاصل کرنے کے لیے ہم ڈومین رینڈمائزیشن استعمال کرتے ہیں:

```bash
# مختلف ماحولیات میں تربیت
for episode in range(num_episodes):
  # ماحولیاتی پیرامیٹرز کو بے ترتیب کریں
  random_friction = random.uniform(0.1, 1.0)
  random_mass = random.uniform(0.8, 1.2)
  random_lighting = random.uniform(0.5, 1.5)
  
  # سیمیولیشن میں ان پیرامیٹرز کو اپ ڈیٹ کریں
  update_environment_params(friction=random_friction, 
                           mass=random_mass, 
                           lighting=random_lighting)
  
  # ایجینٹ کو تربیت دیں
  train_agent()
```

### نقل کے مسائل کو حل کرنا

1. ** ماڈل کی ناکافی مطابقت **: زیادہ درست ماڈل استعمال کریں
2. ** سینسر نوائز **: حقیقی نوائز کے ماڈل شامل کریں
3. ** ڈیٹا کی کمی **: ڈیٹا کو بڑھانے کی تکنیک استعمال کریں

## کارکردگی کی بہتری

### 1. سیمیولیشن کی میٹرکس

- سیمیولیشن کی کارکردگی کو ماپنے کے لیے:
  - اوسط کارکردگی
  - کم سے کم/زیادہ کارکردگی
  - تربیت کا وقت
  - کارکردگی کی تنوع

### 2. مسئلہ کو حل کرنا

- کم کارکردگی والے ماڈل کو شناخت کریں
- کارکردگی کو بہتر بنانے کے لیے پیرامیٹرز اپ ڈیٹ کریں
- سیمیولیشن کی ترتیبات کو بہتر بنائیں

## استعمال کے مواقع

### 1. روبوٹ کنٹرول کی تربیت

```python
# RL ایجینٹ کی تربیت کے لیے سیمیولیشن
import gym
import rospy
from stable_baselines3 import PPO

# خود کار سیمیولیشن ماحول
class RobotControlEnv(gym.Env):
    def __init__(self):
        super(RobotControlEnv, self).__init__()
        # action_space اور observation_space کو بیان کریں
        
    def step(self, action):
        # action کو سیمیولیشن میں لاگو کریں
        # observation، reward، done، info لوٹائیں
        pass
    
    def reset(self):
        # سیمیولیشن کو ری سیٹ کریں
        pass

# RL کا استعمال کرتے ہوئے کنٹرول پالیسی تربیت دیں
env = RobotControlEnv()
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=10000)

# تربیت یافتہ پالیسی کو حقیقتی روبوٹ میں استعمال کریں
```

### 2. نیویگیشن الگورتھم کی جانچ

```bash
# نیویگیشن الگورتھم کی سیمیولیشن میں جانچ
ros2 launch nav2_bringup tb3_simulation_launch.py

# میپ لوڈ کریں
ros2 run map_server map_saver_cli -f ~/map

# نیویگیشن شروع کریں
ros2 launch nav2_bringup navigation_launch.py
```

### 3. SLAM الگورتھم کی جانچ

```bash
# SLAM کے لیے سیمیولیشن
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch slam_toolbox online_sync.launch.py

# میپ کو محفوظ کریں
ros2 run map_server map_saver_cli -f ~/slam_map
```

## خلاصہ

یہ ماڈیول روبوٹ سیمیولیشن کی تکنیکوں کا خلاصہ فراہم کرتا ہے، بشمول:

1. Gazebo اور دیگر سیمیولیشن ماحول کا تعارف
2. ROS 2 کے ساتھ سیمیولیشن کا انضمام
3. سیمیولیشن کی چیلنجز اور حل
4. ڈومین رینڈمائزیشن اور Sim-to-Real ٹرانسفر
5. سیمیولیشن کی کارکردگی کی بہتری
6. استعمال کے مواقع

سیمیولیشن روبوٹکس کی ترقی، جانچ، اور تربیت کے لیے ایک اہم اوزار ہے، جو کم لاگت اور اعلی تحفظ کے ساتھ تجربات کی اجازت دیتا ہے۔