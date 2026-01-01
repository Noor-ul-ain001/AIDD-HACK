---


sidebar_position: 1
difficulty: beginner


---

# 3.1: سیمیولیشن انضمام کی تکنیکیں

## جائزہ

یہ ذیلی ماڈیول ROS 2 کے ساتھ سیمیولیشن انضمام کی تکنیکوں کا جائزہ لیتا ہے، خاص طور پر Gazebo اور دیگر سیمیولیشن ماحول کے ساتھ کام کرنا۔ ہم ROS 2 اور سیمیولیشن کے انضمام کے مختلف پہلوؤں کو سمجھیں گے۔

## سیکھنے کے مقاصد

اس ذیلی ماڈیول کے اختتام تک، آپ:

- ROS 2 اور سیمیولیشن کے انضمام کے طریقے سیکھیں گے
- Gazebo اور ROS 2 کے انضمام کے طریقے سمجھیں گے
- مختلف سیمیولیشن پلیٹ فارم کے موازنہ کریں گے
- ROS 2 سیمیولیشن کے لیے بہترین مشقیں سیکھیں گے
- سیمیولیشن کے مسائل کو حل کرنا سیکھیں گے

## سیمیولیشن انضمام کا تعارف

### براہ راست انضمام بمقابلہ پل ایکریشن

سیمیولیشن اور ROS 2 کے مابین انضمام کے دو بنیادی طریقے ہیں:

#### 1. براہ راست انضمام

کچھ سیمیولیشن انجن (جیسے Gazebo، Isaac Sim) براہ راست ROS 2 API کی حمایت کرتے ہیں:

```python
# مثال: Gazebo میں براہ راست ROS 2 انضمام
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GazeboDirectIntegration(Node):
    def __init__(self):
        super().__init__('gazebo_direct_integration')
        
        # Gazebo میں روبوٹ کنٹرول کے لیے ٹاپکس
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Gazebo سے سینسر ڈیٹا حاصل کرنا
        self.scan_publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

    def cmd_vel_callback(self, msg):
        # Gazebo میں کمانڈ بھیجیں
        # Gazebo کے اندر ہی ROS 2 کنٹرول
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GazeboDirectIntegration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

#### 2. پل ایکریشن

کچھ سیمیولیشن انجن (جیسے PyBullet، Mujoco) کے لیے ROS 2 کے ساتھ مواصلات کے لیے مواصلاتی پل کی ضرورت ہوتی ہے:

```python
# مثال: PyBullet کے لیے پل ایکریشن
import pybullet as p
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SimulationBridge(Node):
    def __init__(self):
        super().__init__('simulation_bridge')
        
        # ROS 2 ٹاپکس کے لیے subscribers اور publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )
        
        # PyBullet سرور کو منسلک کریں
        self.physics_client = p.connect(p.GUI)
        
        # سیمیولیشن لوپ کے لیے ٹائمر
        self.timer = self.create_timer(0.05, self.simulation_loop)

    def cmd_vel_callback(self, msg):
        # PyBullet میں کمانڈز کو تبدیل کریں
        # msg.linear.x اور msg.angular.z کا استعمال کریں
        pass

    def simulation_loop(self):
        # PyBullet کے ایک سٹیپ
        p.stepSimulation()
        
        # سینسر ڈیٹا حاصل کریں اور ROS 2 ٹاپکس پر شائع کریں
        scan_data = self.get_laser_scan()
        self.publish_scan_data(scan_data)

    def get_laser_scan(self):
        # PyBullet سے لیزر اسکین ڈیٹا حاصل کریں
        pass

    def publish_scan_data(self, scan_data):
        # ROS 2 ٹاپک پر اسکین ڈیٹا شائع کریں
        pass

def main(args=None):
    rclpy.init(args=args)
    bridge = SimulationBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()
        bridge.destroy_node()
        rclpy.shutdown()
```

## Gazebo اور ROS 2 کا انضمام

### ros_gz_pkgs کا استعمال

Gazebo اور ROS 2 کے انضمام کے لیے ros_gz_pkgs استعمال کیا جاتا ہے:

- `ros_gz_bridge`: ROS 2 اور Gazebo پیغامات کے مابین ترجمہ
- `ros_gz_sim`: Gazebo سیمیولیشن کے لیے ROS 2 APIs
- `gz_ros2_control`: ROS 2 کنٹرول کا استعمال کرتے ہوئے Gazebo میں سینسر اور ایکٹویٹر

### Gazebo سیمیولیشن کو ROS 2 کے ساتھ شروع کرنا

```bash
# Gazebo سیمیولیشن کو ros2 کے ساتھ شروع کریں
ros2 launch ros_gz_sim gz_sim.launch.py world_name:=empty.sdf

# یا اس کے ذریعے
ros2 run ros_gz_sim gz_sim empty.sdf
```

### مثال: Gazebo میں مائع کا اضافہ

```xml
<!-- مائع کا اضافہ کرنے کے لیے SDF فائل -->
<sdf version="1.7">
  <model name="my_robot">
    <!-- مائع کی تفصیل -->
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.2</size>
          </box>
        </geometry>
      </collision>
      
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
      
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <topic>imu</topic>
      </sensor>
    </link>
    
    <!-- چکر -->
    <joint name="wheel_joint" type="continuous">
      <parent>chassis</parent>
      <child>wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>
    
    <link name="wheel">
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
      </visual>
    </link>
  </model>
</sdf>
```

### Gazebo کنٹرول کے لیے ROS 2 میسجس

Gazebo کے ساتھ ROS 2 میسجس کے لیے مختلف ٹاپکس دستیاب ہیں:

- `/model/{model_name}/cmd_vel`: روبوٹ کو کنٹرول کرنے کا ٹاپک
- `/model/{model_name}/odometry`: روبوٹ کی رفتار کی معلومات
- `/model/{model_name}/joint_states`: جوڑوں کی معلومات
- `/scan`: لیزر اسکینر کے ڈیٹا
- `/camera/image_raw`: کیمرہ کے ڈیٹا

## دیگر سیمیولیشن ماحول

### Isaac Sim

Isaac Sim NVIDIA کا ایک اعلی معیار والی سیمیولیشن ہے:

- فوٹو وریلسٹک رینڈرنگ
- GPU accelerated فزکس (PhysX)
- ROS 2 انضمام کے لیے پل

```python
# Isaac Sim ROS 2 انضمام کا مثال کوڈ
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
import rclpy
from geometry_msgs.msg import Twist

# Isaac Sim میں ROS 2 کنٹرول
class IsaacSimROSController:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        
        # Isaac Sim میں ROS 2 نوڈ کو چلائیں
        rclpy.init()
        self.node = rclpy.create_node('isaac_sim_controller')
        
        # کمانڈ ویل سبسکرائب کریں
        self.cmd_vel_sub = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Isaac Sim کو لانچ کریں
        self.setup_simulation_env()

    def cmd_vel_callback(self, msg):
        # روبوٹ کو Isaac Sim میں کمانڈز بھیجیں
        pass

    def setup_simulation_env(self):
        # Isaac Sim میں ماحول کو سیٹ اپ کریں
        pass
```

### PyBullet

PyBullet فزکس سیمیولیشن کے لیے استعمال ہوتا ہے:

- GPU acceleration
- سافٹ باڈی سیمیولیشن
- ڈیپ لرننگ کے لیے سہولت

```python
import pybullet as p
import pybullet_data
import numpy as np

class PyBulletROSIntegration:
    def __init__(self):
        # PyBullet سرور کو GUI کے ساتھ شروع کریں
        self.physicsClient = p.connect(p.GUI)
        
        # ڈیٹا تلاش کے لیے پاتھ شامل کریں
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # ٹیبل شامل کریں
        self.plane_id = p.loadURDF("plane.urdf")
        
        # روبوٹ کو لوڈ کریں (URDF)
        self.robot_start_pos = [0, 0, 1]
        self.robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = p.loadURDF("r2d2.urdf", 
                                   self.robot_start_pos, 
                                   self.robot_start_orientation)
        
        # فزکس کی ترتیبات
        p.setGravity(0, 0, -9.81)
        self.time_step = 1./240.  # 240 Hz
        p.setTimeStep(self.time_step)

    def step_simulation(self):
        p.stepSimulation()

    def get_robot_state(self):
        # روبوٹ کی موجودہ حالت حاصل کریں
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        return pos, orn

    def apply_control_commands(self, linear_vel, angular_vel):
        # کمانڈز کو روبوٹ پر لاگو کریں
        pass

    def disconnect(self):
        p.disconnect()
```

## سیمیولیشن کے مسائل اور حل

### 1. حقیقت کا فرق

سیمیولیشن اور حقیقتی دنیا کے درمیان فرق:

- فزکس کے ماڈلز کی ناکافی مطابقت
- سینسر نوائز کا نقل ناکام
- کمپیوٹیشنل حدود
- ماحولیاتی متغیرات کا فقدان

#### حل:

1. ** ڈومین رینڈمائزیشن **: سیمیولیشن میں مختلف ماحول کو تبدیل کریں

```python
import random

class DomainRandomization:
    def __init__(self):
        self.params_range = {
            'friction': (0.1, 1.0),
            'mass': (0.5, 1.5),
            'restitution': (0.0, 0.5)
        }

    def randomize_params(self):
        random_params = {}
        for param, (min_val, max_val) in self.params_range.items():
            random_params[param] = random.uniform(min_val, max_val)
        return random_params
```

2. ** ٹرانسفر لرننگ **: سیمیولیشن میں تربیت، حقیقت میں استعمال

3. ** سیم ٹو ریئل کے لیے بہترین مشقیں **: ڈیٹا کی مقدار بڑھائیں

### 2. کارکردگی کے مسائل

- بڑے ماحول کی چلانے کی مشکل
- حقیقی وقت کی کارکردگی کے لیے سرور کی ضرورتیں
- متعدد روبوٹس کے لیے مسائل

#### حل: 

1. ** LOD (Level of Detail) **: کم کارکردہ چیزوں کے لیے سادہ ماڈلز

2. ** QoS (Quality of Service) **: سیمیولیشن کے لیے مناسب QoS ترتیبات

3. ** سیمیولیشن کی فریکوєنسی **: ٹائم سٹیپ کو اپنی ضروریات کے مطابق ایڈجسٹ کریں

### 3. انضمام کے مسائل

- ROS 2 اور سیمیولیشن کے درمیان ترسیل کا نقصان
- پیغام کا زیادہ وقت گزرنے کا مسئلہ
- سینسر ڈیٹا کی ترسیل کے مسائل

#### حل:

```python
# QoS کی مناسب ترتیبات
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ہائی فریکوئینسی ڈیٹا کے لیے
high_freq_qos = QoSProfile(
    depth=1,  # کم ڈیپتھ تاکہ زیادہ میموری نہ لے
    reliability=ReliabilityPolicy.BEST_EFFORT,  # بہترین کوشش
    durability=DurabilityPolicy.VOLATILE  # غیر پائیدار
)

# کریٹیکل ڈیٹا کے لیے
critical_qos = QoSProfile(
    depth=10,  # زیادہ ڈیپتھ
    reliability=ReliabilityPolicy.RELIABLE,  # قابل اطمان
    durability=DurabilityPolicy.TRANSIENT_LOCAL  # مقامی منتقل
)
```

## بہترین مشقیں

### 1. سیمیولیشن کی ترتیبات

1. ** سیمیولیشن کا ٹائم سٹیپ **: ROS 2 نوڈ کی فریکوئینسی کے ساتھ مطابقت رکھیں
2. ** QoS پالیسیز **: ڈیٹا کی قسم کے مطابق مناسب پالیسیز
3. ** سینسر ترتیبات **: حقیقی سینسر کے مطابق

### 2. ماڈل کی تیاری

1. ** URDF/SDF ماڈلز **: سیمیولیشن اور حقیقت کے لیے مناسب
2. ** فزکس پیرامیٹرز **: حقیقی والی قدریں استعمال کریں
3. ** سینسر ماڈلز **: نوائز اور حدود کے ساتھ

### 3. ROS 2 انضمام

1. ** میسج فورمیٹس **: سیمیولیشن اور ROS 2 کے لیے مناسب
2. ** ٹاپک نام **: معیار کے مطابق
3. ** TF ٹریز **: دونوں کے لیے مناسب

## ٹیسٹنگ اور توثیق

### Gazebo کے لیے ٹیسٹنگ

```bash
# Gazebo سیمیولیشن کو ٹیسٹ کریں
ros2 launch ros_gz_sim gz_sim.launch.py world_name:=empty.sdf

# روبوٹ کو چلانے کے لیے مثال کمانڈ
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
```

### Isaac Sim کے لیے ٹیسٹنگ

```bash
# Isaac Sim میں ROS 2 انضمام کو ٹیسٹ کریں
# Isaac Sim میں ROS2 Bridge ایکسٹنشن کو فعال کریں
# Extensions -> Isaac ROS2 Bridge -> Enable
```

### PyBullet کے لیے ٹیسٹنگ

```bash
# PyBullet + ROS 2 کو ٹیسٹ کریں
python3 pybullet_ros_bridge.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## استعمال کی مثالیں

### 1. نیویگیشن کے لیے Gazebo

```bash
# Gazebo میں نیویگیشن ٹیسٹ کریں
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

### 2. SLAM کے لیے Isaac Sim

```bash
# Isaac Sim میں SLAM ٹیسٹ کریں
# Isaac Sim میں روبوٹ لانچ کریں
# ros2 launch slam_toolbox online_async_launch.py
```

### 3. کنٹرول کے لیے PyBullet

```python
# PyBullet میں کنٹرول الگورتھم ٹیسٹ کریں
# PID کنٹرول کا مثال کوڈ
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
```

## خلاصہ

سیمیولیشن انضمام روبوٹکس کی ترقی کے لیے اہم ہے، جس میں:

1. سیمیولیشن اور ROS 2 کے درمیان کم ڈیلے کے ساتھ مواصلات
2. معیار کے مطابق میسج فورمیٹس
3. مناسب سینسر ماڈلز
4. فزکس کی درست ترتیبات
5. کارکردگی کی بہتری کے لیے بہترین مشقیں

سیمیولیشن انضمام کے ذریعے ہم کم خطرے اور کم لاگت میں روبوٹکس الگورتھم کی ترقی، جانچ اور تربیت کر سکتے ہیں۔