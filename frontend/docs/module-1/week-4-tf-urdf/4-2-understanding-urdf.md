---
sidebar_position: 2
difficulty: beginner
---

# 4.2: Understanding URDF (Unified Robot Description Format)

## Overview

This submodule introduces URDF (Unified Robot Description Format), the XML-based format used in ROS to describe robot models. URDF defines the kinematic and dynamic properties of robots, including links, joints, and visual/physical properties.

## Learning Objectives

By the end of this submodule, you will:
- Understand the structure and components of URDF files
- Create URDF descriptions for simple and complex robots
- Define links, joints, and their properties in URDF
- Work with visual, collision, and inertial properties
- Generate TF frames from URDF using robot_state_publisher
- Debug common URDF issues

## URDF Fundamentals

### What is URDF?

URDF (Unified Robot Description Format) is an XML-based file format used in ROS to describe robot models. It defines:

- **Links**: Rigid parts of the robot (e.g., chassis, wheel, sensor)
- **Joints**: Connections between links (e.g., revolute, prismatic, fixed)
- **Visual properties**: How the robot appears in simulation
- **Collision properties**: How the robot interacts physically
- **Inertial properties**: Mass, center of mass, and inertia for physics simulation
- **Materials**: Colors and textures for visualization

### URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
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
</robot>
```

## Link Definition

### Links Components

A link in URDF represents a rigid body and can contain:

1. **Visual**: Defines how the link looks in visualization
2. **Collision**: Defines the physical collision properties
3. **Inertial**: Defines mass and inertia properties for physics simulation

### Basic Link Example

```xml
<link name="base_link">
  <!-- Visual properties -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Box with dimensions 0.5m x 0.3m x 0.1m -->
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

### Geometry Types

URDF supports several geometry types:

- **Box**: `<box size="x y z"/>`
- **Cylinder**: `<cylinder radius="r" length="l"/>`
- **Sphere**: `<sphere radius="r"/>`
- **Mesh**: `<mesh filename="package://path/to/mesh.stl" scale="x y z"/>`

## Joint Definition

### Joint Types

URDF supports several joint types:

1. **Fixed**: No movement between links
2. **Revolute**: Single axis rotation with limits
3. **Continuous**: Continuous rotation (like a wheel)
4. **Prismatic**: Single axis translation with limits
5. **Planar**: Movement in a plane
6. **Floating**: 6DOF movement

### Joint Examples

```xml
<!-- Fixed joint (no movement) -->
<joint name="sensor_mount" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Revolute joint (rotating with limits) -->
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

## Materials and Colors

### Material Definition

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

## Complete Robot Example

### Simple Differential Drive Robot

```xml
<?xml version="1.0"?>
<robot name="diff_drive_robot">
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
  
  <!-- Left wheel -->
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
  
  <!-- Right wheel -->
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
</robot>
```

## URDF with Xacro

### Introduction to Xacro

Xacro is a macro language for generating URDF files. It allows:

- Parameterization of URDF models
- Macro definitions
- Mathematical expressions
- File inclusion

### Basic Xacro Example

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">
  
  <!-- Define properties -->
  <xacro:property name="base_width" value="0.3"/>
  <xacro:property name="base_length" value="0.4"/>
  <xacro:property name="base_height" value="0.1"/>
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_width" value="0.04"/>
  <xacro:property name="PI" value="3.1415926535897931"/>
  
  <!-- Define a macro for wheels -->
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
  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>
  
</robot>
```

## Robot State Publisher

### Using Robot State Publisher

The `robot_state_publisher` node takes a URDF and joint positions to publish the resulting transforms to TF:

```bash
# Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="path_to_urdf"
```

### Node Implementation Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Create a publisher for joint states
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to publish states
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
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joint_states'
        
        # Publish the message
        self.joint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Working with Mesh Files

### Including 3D Models

URDF can reference external 3D models (STL, DAE, etc.):

```xml
<link name="gripper_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Reference to an STL file in the mesh package -->
      <mesh filename="package://my_robot_description/meshes/gripper.stl" scale="1 1 1"/>
    </geometry>
    <material name="light_grey"/>
  </visual>
  
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://my_robot_description/meshes/gripper.stl" scale="1 1 1"/>
    </geometry>
  </collision>
</link>
```

### Creating Collada (DAE) Files

For more complex visual models with textures:
```xml
<visual>
  <geometry>
    <mesh filename="package://my_robot_description/meshes/complex_model.dae"/>
  </geometry>
</visual>
```

## URDF Tools and Validation

### URDF Validation

```bash
# Check URDF validity
check_urdf /path/to/robot.urdf

# Or use xacro to check:
ros2 run xacro xacro -o output.urdf input.urdf.xacro
```

### Viewing URDF

```bash
# View the robot model
ros2 run rviz2 rviz2

# In RViz, add a RobotModel display and set the robot description parameter
# Alternatively, use the command line viewer:
ros2 launch urdf_tutorial display.launch.py model:=path/to/robot.urdf
```

## Robot Description Launch

### Creating a Launch File

```python
# launch/robot_description.launch.py
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindPackageShare("my_robot_description"), "urdf", "robot.urdf.xacro"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([
        node_robot_state_publisher
    ])
```

## Common URDF Issues and Solutions

### 1. Missing Joint Limits (for non-continuous joints)

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

### 2. Incorrect Mass Properties

```xml
<!-- Problem: Zero mass or inertia -->
<link name="problem_link">
  <inertial>
    <mass value="0"/>  <!-- Zero mass is invalid -->
    <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>  <!-- Zero inertia is invalid -->
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

### 3. Floating Robot (no fixed base)

URDF should have a single root link (typically `base_link`):

```xml
<!-- Make sure you have one root link with no parent -->
<link name="base_link">
  <!-- Definition -->
</link>
<!-- All other links connect to this or descendants of this -->
```

### 4. Transform Chain Issues

```xml
<!-- Problem: Disconnected links in the kinematic chain -->
<!-- Solution: All links must be connected through joints -->
```

## Advanced URDF Features

### Gazebo-Specific Tags

For simulation in Gazebo:

```xml
<link name="sensor_link">
  <visual>
    <!-- Visual properties -->
  </visual>
  
  <!-- Gazebo-specific properties -->
  <gazebo reference="sensor_link">
    <material>Gazebo/Blue</material>
    <turnGravityOff>false</turnGravityOff>
  </gazebo>
</link>

<gazebo>
  <!-- Gazebo plugin for sensors -->
  <plugin name="sensor_plugin" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>imu</namespace>
      <remapping>~/out:=imu_data</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
</gazebo>
```

## URDF Best Practices

1. **Use consistent naming**: Follow ROS conventions (`base_link`, `base_footprint`, etc.)
2. **Parameterize with Xacro**: Use xacro for reusability and maintainability
3. **Validate regularly**: Check your URDF with `check_urdf` command
4. **Realistic inertial values**: Use proper mass and inertia properties for simulation
5. **Collision vs. visual**: Use simpler geometry for collision than visual when possible
6. **Origin consistency**: Keep origins consistent between visual, collision, and inertial elements

## Summary

This submodule covered the fundamentals of URDF (Unified Robot Description Format):

- Structure and components of URDF files
- Defining links, joints, and their properties
- Working with visual, collision, and inertial properties
- Using Xacro for parameterized robot descriptions
- Publishing robot states to TF using robot_state_publisher
- Common issues and best practices

URDF is essential for representing robots in ROS, enabling visualization, simulation, and the generation of TF frames. In the next submodule, we'll explore how TF2 and URDF work together to create complete robot representations.