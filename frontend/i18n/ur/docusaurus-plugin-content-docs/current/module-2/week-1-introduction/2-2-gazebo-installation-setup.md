---


sidebar_position: 2
difficulty: intermediate


---
# 2.2: گیز BW tnaib vor atrtaib کے lli ros 2

## ج a ج

الل لِلی لِل ، نعدع ہ ہ کے کے کے کے کے slas las las laus laus laus laus jais jis jais jais jais alaur ssaun گ اوسوس 2 اِن ساسٹ 2

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- anssacasal ک r یں گیز بو ما a یک ss ے ے زی ad ہ taclaat (گیز Bo ک Lasi یکی یکی ، ایڈنا یش n گیز Bo)
- تال ک گیز گیز گیز BO کے l یے یے یے یے یے یے ک ک کے saat ھ ros ros ros ros 2
- امعو کہ بوبو کے
- Test basic گیزبو functionality کے ساتھ ROS 2
.

## گیز بوبو

### گیز بون وورن

WAUA ں ہیں ہیں ہیں ہیں گیز گیز A گیز BO MATAL قہ ک ک ک BO ROS 2 JUCULIDUMN ٹ:

1.
2.

کے l گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز ش گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز ش گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز کی گیز گیز کی گیز گیز گیز گیز کی گیز کی کی کی کی کی گیز

### گیز bw آ r کیٹیکچ r کیٹیکچ r کیٹیکچ r کیٹیکچ r کیٹیکچ r کیٹیکچ r کیٹیکچ r کیٹیکچ r کیٹیکچ m

گیز bi ی ی کئی کئی کئی ک ک l ی d ی ش aaus ش aml ہیں:

.
- **گیزبو Client**: Graphical interface کے لیے visualization
.
.
- ** لائبریریاں **: کور سمولن لائبریریاں

## گیز BO ک LAS یکی ESNSACAHAL ک CHAL -arna (گیز BO 11)

### اوبنٹو اوتب

یے لِل اوبنٹو 22.04 (جی mi ی) کے saaٹ ھ revs rews 2 ia:
```bash
# Add کا/کی OSRF APT repository
sudo apt update && sudo apt install wget
sudo wget https://packages.osrfoundation.org/گیزبو.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$ signed-کے ذریعے=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/گیزبو/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/گیزبو-stable.list > /dev/null

# Update APT cache
sudo apt update

# Install گیزبو 11
sudo apt install gazebo11 libgazebo11-dev
```
### گیز BO ROS پیکیجز کے ll یے ک LAS یکی ک LAS یکی ANSSACAHAL ARNAA
```bash
# Install ROS 2 گیزبو packages
sudo apt install ros-humble-گیزبو-ros-pkgs ros-humble-گیزبو-plugins ros-humble-گیزبو-dev
```
## اِنیشن گیز بوب اِنسال -

### اوبنٹو اواتنڈ بِلریعہ کے لِل العادن نِلع ہ
```bash
# Add کا/کی OSRF APT repository
sudo apt update && sudo apt install wget
sudo wget https://packages.osrfoundation.org/گیزبو.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$ signed-کے ذریعے=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/گیزبو/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/گیزبو-stable.list > /dev/null

# Update APT cache
sudo apt update

# Install Ignition Fortress
sudo apt install ignition-fortress
```
### mumabdl: Aiِn nِniشni گیز bow گ ari ڈ n jnssacasaal -arna (Aala)
```bash
# کے لیے Ubuntu 22.04 کے ساتھ مزید recent Ignition گیزبو
sudo apt install ignition-garden
```
### ros 2 Aisuni یش ni پیکیجز anssaul -arna

کے L یے L یے ROS 2 ش AA ئ SAT ہ کے SAAT ھ AI گ NI یش N یش N یش N گیز BO:
```bash
# Install ROS 2 Ignition گیزبو packages
sudo apt install ros-humble-ign-ros2-control ros-humble-ign-ros2-control-demos ros-humble-ros-gz
```
## ماماول تل

### mawaval matalaiط کی شattb
Add کا/کی following کو آپ کا `~/.bashrc` کو ensure گیزبو works properly:
```bash
# گیزبو Classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/.گیزبو/models:/usr/share/گیزبو-11/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$HOME/.گیزبو:/usr/share/گیزبو-11

# کے لیے Ignition گیزبو
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$HOME/.ignition/گیزبو:/usr/share/ignition/garden
```
ک ک یہ ت ت ت ت ت ت ت ک ک ک
```bash
source ~/.bashrc
```
### ک a/کی n یٹ vr ک vri فی s کی کی کی کی شکی کی کی کی کی کی کی شکی شکی شکی شکی

پ _ کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی _ _ کی کی کی کی کی کی کی _ _ کی کی کی کی کی کی کی _ _ کی کی کی کی کی کی کی _ _ کی کی کی کی کی کی کی آپ _ _ کی کی کی کی کی کی آپ آپ
```bash
# Check current network تشکیل
ifconfig

# اگر گیزبو fails کو start, try:
echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.bashrc
source ~/.bashrc
```
## ٹی s ٹ n گ گیز bo بوب اوتنب

### ٹی S ٹ N گ گیز BO ک LAS یکی
```bash
# Start گیزبو server (headless)
gzserver --verbose

# میں another ٹرمنل, start گیزبو client (GUI)
gzclient --verbose

# یا start both together
گیزبو --verbose
```
### ٹی saun گ aiauni یش n یش گیز گیز bi ی ja
```bash
# List available Ignition commands
ign --help

# Start Ignition گیزبو کے ساتھ ایک simple world
ign گیزبو shapes.sdf

# یا start کے ساتھ empty world
ign گیزبو -r -v 4 # -r auto-start, -v 4 verbose level
```
## اوسر روس 2 ورک اوسسسیس کی ترتب

### smuli یش n پیکیج ت ت ت ت ت ِک ت

n ی a پیکیج کے کے کے کے آپ ک ک ک ک ک ک a smulun arrwauss Buna ئیں:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_gazebo --dependencies \
  rclcpp \
  std_msgs \
  geometry_msgs \
  sensor_msgs \
  gazebo_ros_pkgs \
  gazebo_plugins \
  gazebo_dev \
  robot_state_publisher \
  joint_state_publisher \
  xacro
```
### پیکیج. xml کے l یے گیز Bo anaumamam
Ensure آپ کا `پیکیج.xml` includes کا/کی necessary dependencies:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<پیکیج format="3">
  <name>my_robot_gazebo</name>
  <version>0.1.0</version>
  <description>سمولیشن پیکیج کے لیے میرا روبوٹ</description>
  <maintainer email="user@مثال.com">User</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>gazebo_ros_pkgs</depend>
  <depend>gazebo_plugins</depend>
  <depend>gazebo_dev</depend>
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  <depend>xacro</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</پیکیج>
```
### cmakelists.txt کے l یے گیز Bo anaumamam
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_gazebo)

  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(joint_state_publisher REQUIRED)
find_package(xacro REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install other resources
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

اگر(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```
## گیز bi ی volas verla Jaulas verl ڈ tacl یق

### bin ی aad ی vrl ڈ ف a ئ l
Create ایک simple world file پر `my_robot_gazebo/worlds/my_world.world`:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- Include ایک model سے گیزبو's model database -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include کا/کی sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Add آپ کا own models یہاں -->
    <model name="my_robot">
      <pose>0 0 1 0 0 0</pose>
      <include>
        <uri>model://my_robot_model</uri>
      </include>
    </model>
    
    <!-- Physics parameters -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```
### -lwnچ ف a ئ l کے l یے گیز بو بو anaumamam
Create ایک launch file پر `my_robot_gazebo/launch/my_robot_gazebo.launch.py`:
```python
import os
سے launch import LaunchDescription
سے launch.ایکشنز import IncludeLaunchDescription
سے launch.launch_description_sources import PythonLaunchDescriptionSource
سے launch.substitutions import LaunchConfiguration
سے launch_ros.ایکشنز import نود
سے launch.substitutions import کمانڈ
سے launch.ایکشنز import ExecuteProcess
سے ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get کا/کی پیکیج directory
    pkg_share = get_package_share_directory('my_robot_gazebo')
    
    # Get کا/کی یوآر ڈی ایف file
    urdf_file = os.path.join
    
    # Get کا/کی world file
    world_file = os.path.join(pkg_share, 'worlds', 'my_world.world')
    
    # Launch گیزبو کے ساتھ کا/کی world
    گیزبو = ExecuteProcess(
        cmd=['گیزبو', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # روبوٹ State پبلشر
    robot_state_publisher = نود(
        پیکیج='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': کمانڈ(['xacro ', urdf_file])}]
    )
    
    # Spawn کا/کی روبوٹ میں گیزبو
    spawn_entity = نود(
        پیکیج='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-ٹاپک', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    return LaunchDescription([
        گیزبو,
        robot_state_publisher,
        spawn_entity
    ])
```
## mautr کہ ط vr پ r aaam ط vr پ r jssaaul ک aiaaul ہ ک crna

### 1.

aaar آپ an ک aa ؤ na ٹ r گ ra فک s ss ے matali ق غ laia ں:
```bash
# Try running کے ساتھ سافٹ ویئر rendering
export LIBGL_ALWAYS_SOFTWARE=1
گیزبو

# یا try کے ساتھ OGRE rendering
export GAZEBO_RENDER_ENGINE=ogre
گیزبو
```
### 2

aaar آپ mautri کہ lla ئ bra ی r ی ss ے mnssli غ laiaua ں ح aaul alaul ari یں biad atndb:
```bash
# Update library cache
sudo ldconfig

# Check اگر libraries ہیں properly linked
ldd $(جس گیزبو)
```
### 3. ROS 2 پ L گ پ پ پ گ گ گ گ گ ہیں ہیں ہیں ہیں ہیں

aaar گیز BOW پ l گ ll کے ll یے ros ros ros 2 llo ڈ n ہیں ک r یں:
```bash
# Check اگر gazebo_ros_pkgs ہیں properly installed
dpkg -l | grep گیزبو

# Check گیزبو plugin path
echo $GAZEBO_PLUGIN_PATH

# اگر path ہے missing, add یہ
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/گیزبو-11/plugins:$GAZEBO_PLUGIN_PATH
```
### 4

aaar آپ آپ آپ aa ز t کی غ غ غ غ la طی a ں ح aa ص l ari یں:
```bash
# Check کا/کی permissions کا گیزبو directories
ls -la ~/.گیزبو

# Fix permissions اگر needed
chmod -R 755 ~/.گیزبو
```
## ros 2 anaumamam کی کی ج ج ج ج ک ک ک ک ک ک ک ک ک ک ک ک

### bin ی aad ی ana ض Mam ٹی SA ٹ

، ، سعد ہ ٹی si ٹ tacl یق ک r یں ک j ک کی کی کی کی ص ص ص ص ص یق یق یق یق
```bash
# Create ایک simple launch directory structure
mkdir -p ~/ros2_ws/src/my_robot_gazebo/launch

# Create ایک simple launch file
```
### ک a/کی کی umamam ٹی si ٹ چ sl raal ہے
```bash
# Build کا/کی workspace
cd ~/ros2_ws
colcon build --packages-select my_robot_gazebo

# Source کا/کی workspace
source install/ترتیب.bash

# Try running گیزبو کے ساتھ ROS 2 plugins
گیزبو --verbose

# میں another ٹرمنل, check اگر گیزبو ٹاپکس ہیں available
ros2 ٹاپک list | grep گیزبو
```
## گیز بو کے l یے l یے زی ad ہ s ے زی زی ad ہ ک ک ک ک کی کی شکی شکی شکی شکی شکی شکی

### ک اراورڈ گی کی Jatratauatbat

iخش bian کaraldگی کs jtrati jtrati ی jtrati ی jtrati j جورٹہبت جور جور ک ri:

1.
2
3.

### نعمون کی ک ک ک ک ک گی گی گی گی tal

maaؤs آپ ک a vrli ف a ئ l:
```xml
<physics name="fast_physics" type="ode">
  <max_step_size>0.01</max_step_size>        <!-- Larger step = faster لیکن less accurate -->
  <real_time_factor>1</real_time_factor>     <!-- 1.0 = real-time -->
  <real_time_update_rate>100</real_time_update_rate>  <!-- Update rate میں Hz -->
</physics>
```
## خ LAA صہ

یہ ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ے ک ک ک ک ک ک ک ک ک ک ے ے ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک /////// ک ک ک

- دونوں گیزبو کلاسک اوش اگنیشن گیزبو انسٹال کرنا
- ماؤل متغیرات کی تشکیل
- - ی biی vo ma a ی taalaualہ۔ ورل ڈز
- اوسر روس 2 پیکیجز کی کی ttttttt tt tt tt tt tt tttatat
- مشترکہ تنب کے مسائل کا ازالہ کرنا

mnasasb گیز bw trt ی b ہے ض rvri ی کے l یے l یے l یے mwaurr rrwobow sswul ی n۔ اس کے بعد ، ہم ک الک ماماول ، ہم انضمام ، روبوٹ کو کنٹرول کرنے ، اوکے کام کرنے والے کے سعتی سینسرز ، ایم آئی/کی سیمولہن میمال شامل ہیں۔
