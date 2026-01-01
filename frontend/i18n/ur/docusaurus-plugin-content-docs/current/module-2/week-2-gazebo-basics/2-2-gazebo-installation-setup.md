---


sidebar_position: 2
difficulty: beginner


---

# 2.2: Gazebo تنصیب اور سیٹ اپ

## جائزہ

یہ ذیلی ماڈیول Gazebo کی تنصیب، تشکیل، اور ROS 2 کے ساتھ مربوط کرنے کے عمل کی تفصیل فراہم کرتا ہے۔

## سیکھنے کے مقاصد

اس ذیلی ماڈیول کے اختتام تک، آپ کریں گے:
- Gazebo کی متعدد ورژن کو سمجھیں گے
- Gazebo کو مختلف آپریٹنگ سسٹم پر تنصیب کریں گے
- ROS 2 انضمام کی تشکیل کریں گے
- Gazebo دنیا کو شروع کریں گے

## Gazebo کی تاریخ اور ورژن

Gazebo ایک مقبول روبوٹ سمیولیشن سافٹ ویئر ہے جو پہلا دیکھنے کے لحاظ سے حقیقت پسندانہ ماحول فراہم کرتا ہے۔

### Gazebo کے مختلف ورژن

1. **Original Gazebo**: 2010 میں شروع کیا گیا
2. **Gazebo Classic**: 11 ورژن تک
3. **Gazebo (Ignition)**: Ros2 Humble کے ساتھ استعمال کیا جاتا ہے
4. **Gazebo Garden**: Ros2 Humble کے ساتھ ابتدائی تجربہ

### ROS 2 انضمام

ROS 2 کے ساتھ Gazebo کے انضمام کے لیے، ROS 2 Gazebo pkgs استعمال کیے جاتے ہیں:

- `ros_gz`: ROS 2 اور Gazebo کے مابین مواصلات کا پل
- `ros_gz_bridge`: ROS 2 اور Gazebo پیغامات کے درمیان ترجمہ
- `gz_ros2_control`: ROS 2 کنٹرول کا استعمال کرتے ہوئے Gazebo میں سینسر اور ایکٹویٹر

## Ubuntu پر Gazebo تنصیب

### Gazebo Garden تنصیب

```bash
# Gazebo APT ذریعہ شامل کریں
sudo apt update && sudo apt install wget lsb-release gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg

# Gazebo APT ذریعہ شامل کریں
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo.list > /dev/null

# Gazebo تنصیب کریں
sudo apt update
sudo apt install gz-garden
```

### ROS 2 Gazebo Packages تنصیب

```bash
# Ros2 Humble کے لیے Gazebo packages تنصیب کریں
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-gz-ros2-control
```

## Windows پر Gazebo تنصیب

Windows کے لیے Gazebo تنصیب WSL2 کے ذریعے کی جاتی ہے:

```bash
# WSL Ubuntu ٹرمنل میں
sudo apt update && sudo apt install wget lsb-release gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo.list > /dev/null
sudo apt update
sudo apt install gz-garden ros-humble-ros-gz ros-humble-ros-gz-bridge
```

## macOS پر Gazebo تنصیب

```bash
# Homebrew کے ذریعے
brew install osrf/simulation/gz-garden
brew install ros-humble-ros-gz ros-humble-ros-gz-bridge
```

## Gazebo اور ROS 2 کا آغاز

### Gazebo ورلڈ چلانا

```bash
# Gazebo ورلڈ چلانا
gz sim

# Ros2 کے ساتھ Gazebo ورلڈ چلانا
ros2 launch ros_gz_sim gz_sim.launch.py world_name:=empty.sdf
```

### Ros 2 Gazebo Bridge شروع کرنا

```bash
# Ros2 Gazebo Bridge چلانا
ros2 run ros_gz_bridge parameter_bridge
```

## Ros 2 Gazebo سمیولیشن کی تشکیل

### Ros 2 Package کے ساتھ Gazebo استعمال کرنا

Ros 2 Package کی تشکیل میں، `CMakeLists.txt` اور `package.xml` میں Gazebo dependencies شامل کریں:

`package.xml` میں:
```xml
<depend>ros_gz_sim</depend>
<depend>ros_gz_bridge</depend>
<depend>gz_ros2_control</depend>
```

### Launch File کے ذریعے Gazebo ورلڈ

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_sim.actions import Create
from ros_gz_sim.launch.gz_launch import GzLaunchDescription

def generate_launch_description():
    world = LaunchConfiguration('world')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Choose one of the world files from `/ros_gz_sim/worlds`'
        ),
        
        # Gazebo Server چلانا
        Node(
            package='ros_gz_sim',
            executable='gzserver',
            arguments=['-s', 'libignition-gazebo-physics-system.so', 
                      '-s', 'libignition-gazebo-user-commands-system.so',
                      world],
            output='screen'
        ),
        
        # Gazebo Client چلانا
        Node(
            package='ros_gz_sim',
            executable='gzclient',
            output='screen'
        )
    ])
```

## Gazebo Models اور Worlds

### Gazebo Models کو انسٹال کرنا

```bash
# Ros2 Gazebo models انسٹال کریں
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim

# یا اس کے ذریعے
ros2 run ros_gz_sim download_models
```

### اپنی تخلیق کردہ Gazebo دنیا کو شامل کرنا

```bash
# Ros2 workspace میں ایک دنیا فولڈر بنائیں
mkdir -p ~/ros2_ws/src/my_robot_gazebo/worlds

# دنیا کے فائل کو Ros2 package میں شامل کریں
# دنیا کے فائل کی مثال:
# my_robot_gazebo/worlds/my_world.sdf
```

## بنیادی Gazebo سمیولیشن کا ڈیمو

### Ros 2 Nod سے Gazebo میں ٹاپکس کا استعمال کرکے ڈیٹا بھیجنا

```python
# Publisher script
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GazeboCommander(Node):
    def __init__(self):
        super().__init__('gazebo_commander')
        self.publisher = self.create_publisher(Twist, '/model/vehicle/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.5
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Gazebo سمیولیشن کی تصدیق

### بنیادی سمیولیشن کی جانچ

1. Gazebo ورلڈ چلائیں:
```bash
ros2 launch ros_gz_sim gz_sim.launch.py world_name:=empty.sdf
```

2. Gazebo میں ایک ماڈل شامل کریں:
```bash
ros2 run ros_gz_sim create -world default -file /usr/share/gazebo-11/worlds/models/ground_plane/model.sdf -z 0
```

3. Ros2 ٹاپکس کو جانچیں:
```bash
ros2 topic list | grep gz
```

## خلاصہ

اس ذیلی ماڈیول نے Gazebo کی تنصیب، ROS 2 انضمام، اور بنیادی سمیولیشن کی تشکیل کا عمل سمجھایا ہے۔ اب آپ Gazebo میں روبوٹ سمیولیشن کے لیے ROS 2 کا استعمال کرنے کے لیے تیار ہیں۔ اگلا ذیلی ماڈیول Gazebo کے بنیادی تصورات اور تکنیکوں کا احاطہ کرے گا۔