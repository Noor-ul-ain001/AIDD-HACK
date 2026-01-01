---
sidebar_position: 2
difficulty: intermediate
---

# 2.2: Gazebo Installation and Setup for ROS 2

## Overview

This submodule provides comprehensive instructions for installing and setting up Gazebo for use with ROS 2. We'll cover the installation process, configuration, and initial testing to ensure proper integration with the ROS 2 ecosystem.

## Learning Objectives

By the end of this submodule, you will:
- Install Gazebo in multiple configurations (Gazebo Classic, Ignition Gazebo)
- Configure Gazebo for use with ROS 2
- Understand Gazebo's architecture and components
- Test basic Gazebo functionality with ROS 2
- Set up the Gazebo environment for robotics development

## Gazebo Overview

### Gazebo Versions

There are two main versions of Gazebo relevant to ROS 2 development:

1. **Gazebo Classic (Gazebo 11 and earlier)**: The legacy version, now in maintenance mode
2. **Ignition Gazebo (Fortress, Garden, Harmonic)**: The next-generation simulation framework

For ROS 2 Humble and newer distributions, Ignition Gazebo is the recommended version, though Gazebo Classic is still supported.

### Gazebo Architecture

Gazebo consists of several key components:

- **Gazebo Server**: Headless physics simulation engine
- **Gazebo Client**: Graphical interface for visualization
- **Gazebo Plugins**: Extensible functionality through plugins
- **Fuel**: Online model and world repository
- **Libraries**: Core simulation libraries

## Installing Gazebo Classic (Gazebo 11)

### Ubuntu Installation

For Ubuntu 22.04 (Jammy) with ROS 2 Humble:

```bash
# Add the OSRF APT repository
sudo apt update && sudo apt install wget
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update APT cache
sudo apt update

# Install Gazebo 11
sudo apt install gazebo11 libgazebo11-dev
```

### Installing Gazebo ROS Packages for Classic

```bash
# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev
```

## Installing Ignition Gazebo (Recommended)

### Ubuntu Installation for Ignition Fortress

```bash
# Add the OSRF APT repository (if not already done)
sudo apt update && sudo apt install wget
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update APT cache
sudo apt update

# Install Ignition Fortress
sudo apt install ignition-fortress
```

### Alternative: Installing Ignition Gazebo Garden (More Recent)

```bash
# For Ubuntu 22.04 with more recent Ignition Gazebo
sudo apt install ignition-garden
```

### Installing ROS 2 Ignition Packages

For ROS 2 Humble with Ignition Gazebo:

```bash
# Install ROS 2 Ignition Gazebo packages
sudo apt install ros-humble-ign-ros2-control ros-humble-ign-ros2-control-demos ros-humble-ros-gz
```

## Environment Configuration

### Setting Environment Variables

Add the following to your `~/.bashrc` to ensure Gazebo works properly:

```bash
# Gazebo Classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/.gazebo/models:/usr/share/gazebo-11/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$HOME/.gazebo:/usr/share/gazebo-11

# For Ignition Gazebo (adjust version as needed)
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$HOME/.ignition/gazebo:/usr/share/ignition/garden
```

To apply these changes:

```bash
source ~/.bashrc
```

### Configuring the Network Interface (if needed)

On some systems, you may need to configure the network interface for Gazebo:

```bash
# Check current network configuration
ifconfig

# If Gazebo fails to start, try:
echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.bashrc
source ~/.bashrc
```

## Testing Gazebo Installation

### Testing Gazebo Classic

```bash
# Start Gazebo server (headless)
gzserver --verbose

# In another terminal, start Gazebo client (GUI)
gzclient --verbose

# Or start both together
gazebo --verbose
```

### Testing Ignition Gazebo

```bash
# List available Ignition commands
ign --help

# Start Ignition Gazebo with a simple world
ign gazebo shapes.sdf

# Or start with empty world
ign gazebo -r -v 4 # -r auto-start, -v 4 verbose level
```

## Setting up ROS 2 Workspaces

### Creating a Simulation Package

Create a new package for your simulation projects:

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

### Package.xml for Gazebo Integration

Ensure your `package.xml` includes the necessary dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_gazebo</name>
  <version>0.1.0</version>
  <description>Simulation package for my robot</description>
  <maintainer email="user@example.com">User</maintainer>
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
</package>
```

### CMakeLists.txt for Gazebo Integration

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Gazebo Model and World Creation

### Basic World File

Create a simple world file at `my_robot_gazebo/worlds/my_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- Include a model from Gazebo's model database -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include the sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Add your own models here -->
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

### Launch File for Gazebo Integration

Create a launch file at `my_robot_gazebo/launch/my_robot_gazebo.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('my_robot_gazebo')
    
    # Get the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    
    # Get the world file
    world_file = os.path.join(pkg_share, 'worlds', 'my_world.world')
    
    # Launch Gazebo with the world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )
    
    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

## Troubleshooting Common Installation Issues

### 1. Gazebo fails to start with graphics errors

If you encounter graphics-related errors:

```bash
# Try running with software rendering
export LIBGL_ALWAYS_SOFTWARE=1
gazebo

# Or try with OGRE rendering
export GAZEBO_RENDER_ENGINE=ogre
gazebo
```

### 2. Shared library errors

If you get shared library linking errors after installation:

```bash
# Update library cache
sudo ldconfig

# Check if libraries are properly linked
ldd $(which gazebo)
```

### 3. ROS 2 plugin not found

If Gazebo plugins for ROS 2 don't load:

```bash
# Check if gazebo_ros_pkgs are properly installed
dpkg -l | grep gazebo

# Check Gazebo plugin path
echo $GAZEBO_PLUGIN_PATH

# If path is missing, add it
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH
```

### 4. Permission issues

If you get permission errors:

```bash
# Check the permissions of Gazebo directories
ls -la ~/.gazebo

# Fix permissions if needed
chmod -R 755 ~/.gazebo
```

## Testing ROS 2 Integration

### Basic Integration Test

Let's create a simple test to verify ROS 2 and Gazebo integration:

```bash
# Create a simple launch directory structure
mkdir -p ~/ros2_ws/src/my_robot_gazebo/launch

# Create a simple launch file
# (We'll use the launch file from above)
```

### Running the Integration Test

```bash
# Build the workspace
cd ~/ros2_ws
colcon build --packages-select my_robot_gazebo

# Source the workspace
source install/setup.bash

# Try running Gazebo with ROS 2 plugins
gazebo --verbose

# In another terminal, check if Gazebo topics are available
ros2 topic list | grep gazebo
```

## Configuring Gazebo for Optimal Performance

### Performance Settings

To optimize Gazebo performance, consider these settings:

1. **Physics Update Rate**: Adjust according to simulation needs
2. **Real-time Factor**: Balance between simulation speed and accuracy
3. **Visual Quality**: Adjust graphics settings based on hardware

### Sample Performance Configuration

In your world file:

```xml
<physics name="fast_physics" type="ode">
  <max_step_size>0.01</max_step_size>        <!-- Larger step = faster but less accurate -->
  <real_time_factor>1</real_time_factor>     <!-- 1.0 = real-time -->
  <real_time_update_rate>100</real_time_update_rate>  <!-- Update rate in Hz -->
</physics>
```

## Summary

This submodule covered the installation and setup of Gazebo for ROS 2, including:

- Installing both Gazebo Classic and Ignition Gazebo
- Configuring environment variables
- Creating Gazebo models and worlds
- Setting up ROS 2 packages for simulation
- Troubleshooting common installation issues

Proper Gazebo setup is essential for effective robot simulation. In the next submodule, we'll explore Gazebo basics including launching simulations, controlling robots, and working with sensors in the simulation environment.