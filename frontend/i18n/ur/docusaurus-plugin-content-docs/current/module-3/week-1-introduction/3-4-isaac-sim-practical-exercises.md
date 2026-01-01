---


sidebar_position: 4
difficulty: intermediate


---
# 3.4: آئزک swm iaزmalی maaئی

## ج a ج

یہ الل l ی آپ آپ '' '' '' 'کv مربو ک ک r یں۔

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- maumul smwlain mamamaؤl کے ssassasٹی rewbobubausss sncessr mamaiuss آئزک ssussusm ataumir ک
- روبو نِنین اوسروس اِسنرل مامایسس آئزک سیسسووم اوسو اوانوس
- ک am کے SAAT ھ SAUM کے ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ی ی
- اِن ی ی ی کے کے کے کے کے کے کے کے کے کے کے کے کے کے آئزک آئزک کے کے آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک
- اِنوومام آئزک سوم کے SAAT ھ ROS 2 N ی OVAU یگیش N ASA
- ڈیBگ ک Amn آئزک Saum iau ز - ک araird گی ک v Biautr Buna ئیں

## vr ک JAS 1: mamaml ط vr پ r پ roboobo smumolain maaasol

###
ia یک یک mauml smulian maamamaw آئزک sausm کے satai کے یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک ، یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک یک

### mracl ہ 1: vri ک یsss ڈھ ڈھ ڈھ ڈھ چہ چہ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ

پہلا ، ، ​​، ک ک ک/کی ض ض ض ض ض ض ض ض buna ئیں buna ئیں lulil ہ lulil ہ ہ maura آئزک ssum mumim:
```bash
# Create ایک directory کے لیے آئزک سیم exercises
mkdir -p ~/isaac_sim_exercises/{robots,worlds,scripts,config}
cd ~/isaac_sim_exercises
```
### mrucl ہ 2: بنا ریوبوبو طال
Create ایک Python script کو set اوپر ہمارا روبوٹ میں آئزک سیم (`~/isaac_sim_exercises/scripts/setup_robot.py`):
```python
#!/usr/bin/env python3
"""
Complete روبوٹ ترتیب script کے لیے آئزک سیم
"""

import sys
import os

# Add آئزک سیم Python paths
isaac_sim_path = os.environ.get('ISAAC_SIM_PATH')
اگر isaac_sim_path:
    sys.path.insert(0, os.path.join(isaac_sim_path, 'python'))
    sys.path.insert(0, os.path.join(isaac_sim_path, 'kit'))

# Import آئزک سیم modules
سے omni.isaac.core import World
سے omni.isaac.core.robots import روبوٹ
سے omni.isaac.core.utils.stage import add_reference_to_stage
سے omni.isaac.core.utils.nucleus import get_assets_root_path
سے omni.isaac.core.utils.prims import create_prim, get_prim_at_path
سے omni.isaac.core.utils.stage import get_stage_units
سے omni.isaac.core.materials import PreviewSurface
سے pxr import Gf, UsdGeom
import numpy کے طور پر np


class IsaacSimExerciseRobot:
    """Class کو manage ہمارا ورک ایکس روبوٹ میں آئزک سیم"""
    
    def __init__(self):
        """Initialize کا/کی ورک ایکس روبوٹ"""
        self.world = None
        self.روبوٹ = None
        self.robot_path = "/World/ExerciseRobot"
        self.name = "exercise_robot"
        
    def setup_world(self):
        """Set اوپر کا/کی سمولیشن world"""
        
        # Create کا/کی world کے ساتھ 1m units
        self.world = World(stage_units_in_meters=1.0)
        
        # Add default ground plane
        self.world.scene.add_default_ground_plane()
        
        
    def add_environment_objects(self):
        """Add ماحول objects کو کا/کی scene"""
        
        # Add ایک table
        create_prim(
            prim_path="/World/Table",
            prim_type="Cuboid",
            position=[2, 0, 0.5],
            scale=[1.5, 1, 0.05]
        )
        
        # Add ایک box پر کا/کی table
        create_prim(
            prim_path="/World/Table/Box",
            prim_type="Cube",
            position=[2, 0, 0.75],
            scale=[0.3, 0.3, 0.3]
        )
        
        # Add ایک wall
        create_prim(
            prim_path="/World/Wall",
            prim_type="Cuboid",
            position=[0, 3, 1],
            scale=[5, 0.1, 2],
            orientation=[0, 0, 0.707, 0.707]  # 90-degree rotation around Z
        )
        
        # Create ایک simple room-like ماحول
        create_prim(
            prim_path="/World/Wall1",
            prim_type="Cuboid",
            position=[3, 0, 1],
            scale=[0.1, 5, 2]
        )
        
    
    def add_robot(self):
        """Add کا/کی روبوٹ کو کا/کی سمولیشن"""
        print("Adding روبوٹ کو سمولیشن...")
        
        try:
            # Try کو use ایک default روبوٹ asset
            assets_root_path = get_assets_root_path()
            اگر assets_root_path:
                robot_usd_path = assets_root_path + "/Isaac/Robots/TurtleBot3Burger/turtlebot3_burger.usd"
                
                self.روبوٹ = self.world.scene.add(
                    روبوٹ(
                        prim_path=self.robot_path,
                        name=self.name,
                        usd_path=robot_usd_path,
                        position=[0, 0, 0.1],
                        orientation=[0, 0, 0, 1]
                    )
                )
            else:
                print("✗ کر سکتا تھا نہیں find آئزک سیم assets, creating ایک basic روبوٹ")
                # Fallback: Create ایک simple روبوٹ using basic prims
                self.create_basic_robot()
                
        except Exception کے طور پر e:
            # Fallback: Create ایک simple روبوٹ
            self.create_basic_robot()
    
    def create_basic_robot(self):
        """Create ایک basic روبوٹ اگر assets ہیں نہیں available"""
        
        # Create روبوٹ root
        create_prim(
            prim_path=self.robot_path,
            prim_type="Xform",
            position=[0, 0, 0.5]
        )
        
        # Create روبوٹ base
        create_prim(
            prim_path=f"{self.robot_path}/Base",
            prim_type="Cylinder",
            position=[0, 0, 0],
            scale=[0.2, 0.2, 0.2]
        )
        
        # Create wheels
        create_prim(
            prim_path=f"{self.robot_path}/LeftWheel",
            prim_type="Cylinder",
            position=[0, 0.15, 0],
            scale=[0.1, 0.1, 0.05]
        )
        
        create_prim(
            prim_path=f"{self.robot_path}/RightWheel",
            prim_type="Cylinder",
            position=[0, -0.15, 0],
            scale=[0.1, 0.1, 0.05]
        )
        
    
    def add_sensors(self):
        """Add sensors کو کا/کی روبوٹ"""
        
        # Add ایک camera sensor
        try:
            سے omni.isaac.sensor import Camera
            
            camera = Camera(
                prim_path=f"{self.robot_path}/FrontCamera",
                position=np.array([0.1, 0, 0.1]),  # Position relative کو روبوٹ
                frequency=30,
                resolution=(640, 480)
            )
            camera.initialize()
            
            print("✓ Camera sensor added")
        except Exception کے طور پر e:
        
        # Add LiDAR sensor
        try:
            سے omni.isaac.sensor import RotatingLidarPhysX
            
            lidar = RotatingLidarPhysX(
                prim_path=f"{self.robot_path}/Lidar",
                translation=np.array([0.05, 0, 0.2]),
                config="Example_Rotary",
                depth_range=10.0,
                horizontal_resolution=0.25,
                vertical_resolution=0.5,
                horizontal_fov=360,
                vertical_fov=30,
                rotation_frequency=10,
                samples_per_scan=1440
            )
            lidar.initialize()
            
            print("✓ LiDAR sensor added")
        except Exception کے طور پر e:
        
        # Add IMU sensor
        try:
            سے omni.isaac.core.sensors import Imu
            
            imu = Imu(
                prim_path=f"{self.robot_path}/Imu",
                position=np.array([0.0, 0.0, 0.1]),
                orientation=np.array([0, 0, 0, 1])
            )
            imu.initialize()
            
            print("✓ IMU sensor added")
        except Exception کے طور پر e:
    
    def setup_physics(self):
        """Configure physics properties کے لیے کا/کی روبوٹ"""
        
        # Set اوپر basic physics کے لیے روبوٹ components
        try:
            سے omni.isaac.core.utils.prims import setRigidBodyProperties
            سے omni.isaac.core.utils.prims import setStaticColliderProperties
            
            # Set properties کے لیے روبوٹ base
            base_path = f"{self.robot_path}/Base"
            setRigidBodyProperties(
                prim_path=base_path,
                mass=10.0,
                linear_damping=0.05,
                angular_damping=0.1
            )
            
            setStaticColliderProperties(
                prim_path=base_path,
                approximation_shape="convexHull"
            )
            
            print("✓ Physics properties configured")
        except Exception کے طور پر e:
    
    def run_simulation(self):
        """Run کا/کی complete سمولیشن"""
        
        # Reset کا/کی world کو initialize تمام objects
        self.world.reset()
        
        print("✓ World reset complete")
        
        # Run سمولیشن کے لیے ایک few steps
        کے لیے میں میں range(100):
            self.world.step(render=True)
            
            اگر میں % 20 == 0:
                # Print روبوٹ position periodically
                اگر self.روبوٹ:
                    pos, quat = self.روبوٹ.get_world_pose()
        
        
        # Cleanup
        self.world.clear()
        print("✓ World cleared")


def main():
    """Main function کو run کا/کی ورک ایکس"""
    print("="*60)
    print("آئزک سیم ورک ایکس: Complete روبوٹ ترتیب")
    print("="*60)
    
    # Create اور run کا/کی ورک ایکس روبوٹ
    exercise_robot = IsaacSimExerciseRobot()
    
    try:
        exercise_robot.setup_world()
        exercise_robot.add_environment_objects()
        exercise_robot.add_robot()
        exercise_robot.add_sensors()
        exercise_robot.setup_physics()
        exercise_robot.run_simulation()
        
        
    except Exception کے طور پر e:
        import traceback
        traceback.print_exc()


اگر __name__ == "__main__":
    main()
```
### Mraul ہ 3: سنسر عمامام یsasareپٹ bunک
Create ایک script کو handle sensor data اور ROS 2 integration (`~/isaac_sim_exercises/scripts/sensor_integration.py`):
```python
#!/usr/bin/env python3
"""
Sensor integration script کے لیے آئزک سیم ورک ایکس
"""

import sys
import os
import numpy کے طور پر np

# Add آئزک سیم paths
isaac_sim_path = os.environ.get('ISAAC_SIM_PATH')
اگر isaac_sim_path:
    sys.path.insert(0, os.path.join(isaac_sim_path, 'python'))
    sys.path.insert(0, os.path.join(isaac_sim_path, 'kit'))

# Import آئزک سیم modules
سے omni.isaac.core import World
سے omni.isaac.core.robots import روبوٹ
سے omni.isaac.core.utils.nucleus import get_assets_root_path
سے omni.isaac.core.utils.stage import add_reference_to_stage
سے omni.isaac.core.utils.prims import get_prim_at_path

# Import sensor modules
سے omni.isaac.sensor import Camera, RotatingLidarPhysX
سے omni.isaac.core.sensors import Imu

# Import ROS 2 bridge
try:
    import rclpy
    سے sensor_msgs.msg import Image, LaserScan, Imu کے طور پر ImuMsg
    سے geometry_msgs.msg import Twist
    سے std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class IsaacSimSensorProcessor:
    """Class کو handle sensor data processing"""
    
    def __init__(self, robot_path="/World/ExerciseRobot"):
        self.robot_path = robot_path
        self.world = None
        self.camera = None
        self.lidar = None
        self.imu = None
        
        # ROS 2 related
        self.ros_node = None
        self.ros_initialized = False
        
        # Sensor data storage
        self.camera_data = None
        self.lidar_data = None
        self.imu_data = None
        
    def initialize_sensors(self):
        """Initialize تمام sensors میں کا/کی سمولیشن"""
        print("Initializing sensors...")
        
        # Initialize sensors اگر they exist
        try:
            # Initialize camera
            camera_path = f"{self.robot_path}/FrontCamera"
            camera_prim = get_prim_at_path(camera_path)
            اگر camera_prim:
                self.camera = Camera(prim_path=camera_path)
                self.camera.initialize()
                print("✓ Camera initialized")
            else:
        
        except Exception کے طور پر e:
        
        try:
            # Initialize LiDAR
            lidar_path = f"{self.robot_path}/Lidar"
            lidar_prim = get_prim_at_path(lidar_path)
            اگر lidar_prim:
                self.lidar = RotatingLidarPhysX(prim_path=lidar_path)
                self.lidar.initialize()
                print("✓ LiDAR initialized")
            else:
        
        except Exception کے طور پر e:
        
        try:
            # Initialize IMU
            imu_path = f"{self.robot_path}/Imu"
            imu_prim = get_prim_at_path(imu_path)
            اگر imu_prim:
                self.imu = Imu(prim_path=imu_path)
                self.imu.initialize()
                print("✓ IMU initialized")
            else:
        
        except Exception کے طور پر e:
    
    def initialize_ros2(self):
        """Initialize ROS 2 مواصلات اگر available"""
        اگر نہیں ROS2_AVAILABLE:
            return False
        
        try:
            rclpy.init()
            self.ros_node = rclpy.create_node('isaac_sim_sensor_processor')
            
            # Create publishers کے لیے sensor data
            self.image_pub = self.ros_node.create_publisher(Image, '/camera/image_raw', 10)
            self.laser_pub = self.ros_node.create_publisher(LaserScan, '/scan', 10)
            self.imu_pub = self.ros_node.create_publisher(ImuMsg, '/imu/data', 10)
            
            # Create سبسکرائیبر کے لیے روبوٹ control
            self.cmd_vel_sub = self.ros_node.create_subscription(
                Twist, '/cmd_vel', self.cmd_vel_callback, 10)
            
            print("✓ ROS 2 initialized")
            self.ros_initialized = True
            return True
            
        except Exception کے طور پر e:
            return False
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands سے ROS 2"""
        # میں ایک real نفاذ, یہ کرے گا control کا/کی روبوٹ
        # کے لیے اب, we'll just print کا/کی کمانڈ
    
    def capture_sensor_data(self):
        """Capture اور process data سے تمام sensors"""
        try:
            # Capture camera data
            اگر self.camera:
                try:
                    self.camera_data = self.camera.get_rgb()
                    print(f"✓ Captured camera data: shape {self.camera_data.shape}")
                    
                    # Publish کو ROS 2 اگر available
                    اگر self.ros_initialized:
                        self.publish_camera_data()
                        
                except Exception کے طور پر e:
        
        except Exception کے طور پر e:
            print(f"✗ Camera capture error: {e}")
        
        try:
            # Capture LiDAR data
            اگر self.lidar:
                try:
                    self.lidar_data = self.lidar.get_linear_depth_data()
                    print(f"✓ Captured LiDAR data: shape {self.lidar_data.shape}")
                    
                    # Publish کو ROS 2 اگر available
                    اگر self.ros_initialized:
                        self.publish_lidar_data()
                        
                except Exception کے طور پر e:
        
        except Exception کے طور پر e:
            print(f"✗ LiDAR capture error: {e}")
        
        try:
            # Capture IMU data
            اگر self.imu:
                try:
                    self.imu_data = {
                        'linear_acceleration': self.imu.get_linear_acceleration(),
                        'angular_velocity': self.imu.get_angular_velocity()
                    }
                    print(f"✓ Captured IMU data: linear_acc={self.imu_data['linear_acceleration']}")
                    
                    # Publish کو ROS 2 اگر available
                    اگر self.ros_initialized:
                        self.publish_imu_data()
                        
                except Exception کے طور پر e:
        
        except Exception کے طور پر e:
            print(f"✗ IMU capture error: {e}")
    
    def publish_camera_data(self):
        """Publish camera data کو ROS 2"""
        اگر نہیں:
            return
        
        try:
            # Convert numpy array کو ROS Image message
            header = Header()
            header.stamp = self.ros_node.get_clock().اب().to_msg()
            header.frame_id = "camera_frame"
            
            # Create Image message
            img_msg = Image()
            img_msg.header = header
            img_msg.height = self.camera_data.shape[0]
            img_msg.width = self.camera_data.shape[1]
            img_msg.encoding = "rgb8"  # TODO: Proper conversion needed
            img_msg.is_bigendian = False
            img_msg.step = self.camera_data.shape[1] * 3  # 3 channels (RGB)
            # img_msg.data = self.camera_data.flatten().tobytes()  # TODO: Proper data conversion
            
            self.image_pub.publish(img_msg)
            
        except Exception کے طور پر e:
    
    def publish_lidar_data(self):
        """Publish LiDAR data کو ROS 2"""
        اگر نہیں:
            return
        
        try:
            # Convert lidar data کو LaserScan message
            laser_msg = LaserScan()
            laser_msg.header.stamp = self.ros_node.get_clock().اب().to_msg()
            laser_msg.header.frame_id = "lidar_frame"
            
            # Define scan parameters
            laser_msg.angle_min = -np.pi  # Assuming 360° scan
            laser_msg.angle_max = np.pi
            laser_msg.angle_increment = 2 * np.pi / len(self.lidar_data) اگر len(self.lidar_data) > 0 else 0.01
            laser_msg.time_increment = 0.0  # Time between measurements
            laser_msg.scan_time = 0.1  # Time between scans
            laser_msg.range_min = 0.1  # Minimum range
            laser_msg.range_max = 10.0  # Maximum range
            
            # Convert distance data کو ranges
            laser_msg.ranges = self.lidar_data.tolist() اگر self.lidar_data ہے نہیں None else []
            laser_msg.intensities = []  # نہیں intensity data کے لیے depth صرف
            
            self.laser_pub.publish(laser_msg)
            
        except Exception کے طور پر e:
    
    def publish_imu_data(self):
        """Publish IMU data کو ROS 2"""
        اگر نہیں:
            return
        
        try:
            # Create IMU message
            imu_msg = ImuMsg()
            imu_msg.header.stamp = self.ros_node.get_clock().اب().to_msg()
            imu_msg.header.frame_id = "imu_frame"
            
            # Set linear acceleration
            imu_msg.linear_acceleration.x = self.imu_data['linear_acceleration'][0]
            imu_msg.linear_acceleration.y = self.imu_data['linear_acceleration'][1]
            imu_msg.linear_acceleration.z = self.imu_data['linear_acceleration'][2]
            
            # کے لیے angular velocity اور orientation, we'd need additional data
            # Set کو zero کے لیے اب
            imu_msg.angular_velocity.x = self.imu_data['angular_velocity'][0]
            imu_msg.angular_velocity.y = self.imu_data['angular_velocity'][1]
            imu_msg.angular_velocity.z = self.imu_data['angular_velocity'][2]
            
            # Orientation unavailable سے یہ IMU API
            imu_msg.orientation.w = 1.0  # Default orientation
            
            self.imu_pub.publish(imu_msg)
            
        except Exception کے طور پر e:
    
    def run_sensor_loop(self, num_steps=200):
        """Run کا/کی sensor processing loop"""
        
        # Initialize world اگر نہیں done already
        اگر نہیں self.world:
            self.world = World(stage_units_in_meters=1.0)
            self.world.scene.add_default_ground_plane()
            
            # Add ایک simple روبوٹ اگر needed
            assets_root_path = get_assets_root_path()
            اگر assets_root_path:
                try:
                    self.world.scene.add(
                        روبوٹ(
                            prim_path="/World/SensorRobot",
                            name="sensor_robot",
                            usd_path=assets_root_path + "/Isaac/Robots/TurtleBot3Burger/turtlebot3_burger.usd",
                            position=[0, 0, 0.1],
                            orientation=[0, 0, 0, 1]
                        )
                    )
                except:
                    سے omni.isaac.core.utils.prims import create_prim
                    create_prim(
                        prim_path="/World/SensorRobot",
                        prim_type="Xform",
                        position=[0, 0, 0.5]
                    )
            
            self.world.reset()
        
        # Initialize sensors اور ROS2
        self.initialize_sensors()
        self.initialize_ros2()
        
        # Main sensor loop
        کے لیے میں میں range(num_steps):
            # Step کا/کی سمولیشن
            self.world.step(render=True)
            
            # Capture sensor data periodically
            اگر میں % 10 == 0:  # Capture data every 10 steps
                self.capture_sensor_data()
                
                # Spin ROS 2 کو process callbacks
                اگر self.ros_node:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.01)
        
        # Cleanup
        اگر self.ros_node:
            self.ros_node.destroy_node()
            rclpy.shutdown()
        
        اگر self.world:
            self.world.clear()
        
        print("✓ Sensor processing loop completed")


def main():
    """Main function کو run sensor integration ورک ایکس"""
    print("="*60)
    print("="*60)
    
    # Create sensor processor
    sensor_processor = IsaacSimSensorProcessor()
    
    try:
        sensor_processor.run_sensor_loop(num_steps=200)
        
    except Exception کے طور پر e:
        import traceback
        traceback.print_exc()


اگر __name__ == "__main__":
    main()
```
## vr ک iass 2: rubbo nidnwsne maaؤs آئزک saum

###
را ہ کی munaubab ہ bund ی ک a ک asatamaal ک Samamal ک Ce Ju ئے bun ی Ad ی ni ی sowauaauaouauon ssissusm mauss آئزک ssum una فذ
Create کا/کی navigation script (`~/isaac_sim_exercises/scripts/navigation_exercise.py`):
```python
#!/usr/bin/env python3
"""
Navigation ورک ایکس script کے لیے آئزک سیم
"""

import sys
import os
import numpy کے طور پر np
import math

# Add آئزک سیم paths
isaac_sim_path = os.environ.get('ISAAC_SIM_PATH')
اگر isaac_sim_path:
    sys.path.insert(0, os.path.join(isaac_sim_path, 'python'))
    sys.path.insert(0, os.path.join(isaac_sim_path, 'kit'))

# Import آئزک سیم modules
سے omni.isaac.core import World
سے omni.isaac.core.robots import روبوٹ
سے omni.isaac.core.utils.nucleus import get_assets_root_path
سے omni.isaac.core.utils.prims import create_prim
سے omni.isaac.core.utils.stage import add_reference_to_stage
سے omni.isaac.core.utils.viewports import get_viewport_window
سے omni.isaac.core.materials import PreviewSurface

# Import sensor modules
سے omni.isaac.sensor import RotatingLidarPhysX


class IsaacSimNavigationExercise:
    """Class کو handle navigation میں آئزک سیم"""
    
    def __init__(self):
        self.world = None
        self.روبوٹ = None
        self.lidar = None
        self.robot_path = "/World/NavigationRobot"
        self.target_position = np.array([3.0, 2.0, 0.0])
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_orientation = 0.0  # Yaw angle میں radians
        
        # Navigation parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.safe_distance = 0.8  # meters
        self.arrival_threshold = 0.3  # meters
        
        # Path planning
        self.path = []
        self.path_index = 0
    
    def setup_world(self):
        """Set اوپر کا/کی navigation world کے ساتھ obstacles"""
        
        # Create کا/کی world
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        
        # Add start اور target positions
        # Start position: (0, 0)
        # Target position: (3, 2)
        
        # Add obstacles
        create_prim(
            prim_path="/World/Obstacle1",
            prim_type="Cylinder",
            position=[1.5, 1.0, 0.5],
            scale=[0.5, 0.5, 1.0]
        )
        
        create_prim(
            prim_path="/World/Obstacle2",
            prim_type="Cube",
            position=[2.0, -1.0, 0.5],
            scale=[0.7, 0.7, 1.0]
        )
        
        create_prim(
            prim_path="/World/Obstacle3",
            prim_type="Cylinder",
            position=[0.5, 2.0, 0.5],
            scale=[0.4, 0.4, 1.0]
        )
        
        # Add target marker
        create_prim(
            prim_path="/World/Target",
            prim_type="Sphere",
            position=[self.target_position[0], self.target_position[1], 0.5],
            scale=[0.3, 0.3, 0.3]
        )
        
        # Get روبوٹ assets اور add کو world
        assets_root_path = get_assets_root_path()
        اگر assets_root_path:
            try:
                self.روبوٹ = self.world.scene.add(
                    روبوٹ(
                        prim_path=self.robot_path,
                        name="navigation_robot",
                        usd_path=assets_root_path + "/Isaac/Robots/TurtleBot3Burger/turtlebot3_burger.usd",
                        position=[0, 0, 0.1],
                        orientation=[0, 0, 0, 1]
                    )
                )
            except:
                # Create ایک basic روبوٹ اگر assets unavailable
                create_prim(
                    prim_path=self.robot_path,
                    prim_type="Xform",
                    position=[0.0, 0.0, 0.5]
                )
        
        # Add LiDAR کو کا/کی روبوٹ
        try:
            self.lidar = RotatingLidarPhysX(
                prim_path=f"{self.robot_path}/Lidar",
                translation=np.array([0.05, 0, 0.2]),
                config="Example_Rotary",
                depth_range=10.0,
                horizontal_resolution=0.25,
                vertical_resolution=0.5,
                horizontal_fov=360,
                vertical_fov=30,
                rotation_frequency=10,
                samples_per_scan=1440
            )
            self.lidar.initialize()
        except Exception کے طور پر e:
        
        # Reset کا/کی world کو initialize تمام objects
        self.world.reset()
        
    
    def get_robot_state(self):
        """Get current روبوٹ state"""
        اگر self.روبوٹ:
            pos, quat = self.روبوٹ.get_world_pose()
            self.current_position = np.array([pos[0], pos[1], pos[2]])
            
            # Convert quaternion کو yaw
            # کے لیے ایک quaternion [x, y, z, w], yaw = atan2(2*(w*z + x*y), w^2 + x^2 - y^2 - z^2)
            w, x, y, z = quat
            self.current_orientation = math.atan2(2*(w*z + x*y), w*w + x*x - y*y - z*z)
        
        return self.current_position, self.current_orientation
    
    def get_lidar_data(self):
        """Get LiDAR data کو detect obstacles"""
        اگر self.lidar:
            try:
                return self.lidar.get_linear_depth_data()
            except:
                # Return empty data اگر LiDAR ہے نہیں available
                return np.ones(360) * 10.0
        else:
            # Return empty data اگر نہیں LiDAR
            return np.ones(360) * 10.0
    
    def check_obstacles_ahead(self, lidar_data):
        """Check کے لیے obstacles میں سامنے کا کا/کی روبوٹ"""
        # Check کا/کی سامنے 30 degrees کا کا/کی LiDAR data (15 points)
        front_start = len(lidar_data) // 2 - 15
        front_end = len(lidar_data) // 2 + 15
        
        اگر front_start < 0:
            front_start = 0
        اگر front_end >= len(lidar_data):
            front_end = len(lidar_data) - 1
        
        # Check اگر any distance ہے less than safe distance
        کے لیے میں میں range(front_start, front_end):
            اگر lidar_data[میں] < self.safe_distance اور نہیں np.isnan:
                return True
        
        return False
    
    def calculate_control_command(self):
        """Calculate control کمانڈ کو navigate کو target"""
        # Get current state
        pos, orientation = self.get_robot_state()
        
        # Calculate direction کو target
        target_delta = self.target_position - pos
        target_distance = np.linalg.norm(target_delta[:2])  # صرف x,y plane
        
        # Check اگر we've reached کا/کی target
        اگر target_distance < self.arrival_threshold:
            return 0.0, 0.0  # Stop moving
        
        # Calculate target angle
        target_angle = np.arctan2(target_delta[1], target_delta[0])
        
        # Calculate angle difference
        angle_diff = target_angle - orientation
        # Normalize angle کو [-π, π]
        جب تک angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        جب تک angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Get LiDAR data کو check کے لیے obstacles
        lidar_data = self.get_lidar_data()
        obstacle_ahead = self.check_obstacles_ahead(lidar_data)
        
        # Navigation logic
        اگر obstacle_ahead:
            # اگر obstacle ahead, turn away
            return 0.0, self.angular_speed  # Rotate کو avoid obstacle
        elif abs(angle_diff) > 0.2:  # اگر نہیں aligned کے ساتھ target
            # Rotate toward target
            angular_cmd = self.angular_speed اگر angle_diff > 0 else -self.angular_speed
            return 0.0, angular_cmd
        else:
            # Move toward target
            return self.linear_speed, 0.0
    
    def execute_navigation(self, max_steps=2000):
        """Execute کا/کی navigation task"""
        print("Starting navigation task...")
        print(f"Target position: [{self.target_position[0]:.2f}, {self.target_position[1]:.2f}]")
        
        reached_target = False
        
        کے لیے step میں range(max_steps):
            # Step کا/کی سمولیشن
            self.world.step(render=True)
            
            # Calculate control commands every few steps کے لیے efficiency
            اگر step % 5 == 0:
                linear_vel, angular_vel = self.calculate_control_command()
                
                # Apply control
                # کے لیے یہ ورک ایکس, we'll just print کا/کی commands
                print(f"Step {step}: v={linear_vel:.2f}, ω={angular_vel:.2f}")
                
                # Check اگر we've reached کا/کی target
                pos, _ = self.get_robot_state()
                distance_to_target = np.linalg.norm(self.target_position[:2] - pos[:2])
                
                اگر distance_to_target < self.arrival_threshold:
                    print(f"✓ Reached target! Final distance: {distance_to_target:.2f}m")
                    reached_target = True
                    break
            
            # Print status periodically
            اگر step % 100 == 0:
                pos, _ = self.get_robot_state()
                distance_to_target = np.linalg.norm(self.target_position[:2] - pos[:2])
        
        اگر نہیں reached_target:
            pos, _ = self.get_robot_state()
            distance_to_target = np.linalg.norm(self.target_position[:2] - pos[:2])
            print(f"⚠ Navigation ended without reaching target. Final distance: {distance_to_target:.2f}m")
        
        print("✓ Navigation task completed")
    
    def run_exercise(self):
        """Run کا/کی complete navigation ورک ایکس"""
        print("="*60)
        print("آئزک سیم ورک ایکس: روبوٹ Navigation")
        print("="*60)
        
        try:
            self.setup_world()
            self.execute_navigation(max_steps=2000)
            
            # Cleanup
            self.world.clear()
            
            
        except Exception کے طور پر e:
            import traceback
            traceback.print_exc()


def main():
    """Main function کو run navigation ورک ایکس"""
    nav_exercise = IsaacSimNavigationExercise()
    nav_exercise.run_exercise()


اگر __name__ == "__main__":
    main()
```
## vr ک iass 3: aiaunsi ڈ saunsr jrosossn گ

###
ا کی جے جے جےsncr پsncr پsnsr پsrossssnaiگ کv juaفذ کsrیں s ssusm ssaunssr کasamasl کasamal ssulauئے ssulauئے sslauc sslaus sslaus sulauchaus sslaus sulaus sslaus sulaus سیسلاؤس سلاؤس سولوس سولوس سولوس سولوس سولوس سولوک ساسامال ساسمل ساسامال ساساماس
Create کا/کی advanced sensor script (`~/isaac_sim_exercises/scripts/advanced_sensor_exercise.py`):
```python
#!/usr/bin/env python3
"""
Advanced sensor processing ورک ایکس کے لیے آئزک سیم
"""

import sys
import os
import numpy کے طور پر np
import math
سے collections import deque

# Add آئزک سیم paths
isaac_sim_path = os.environ.get('ISAAC_SIM_PATH')
اگر isaac_sim_path:
    sys.path.insert(0, os.path.join(isaac_sim_path, 'python'))
    sys.path.insert(0, os.path.join(isaac_sim_path, 'kit'))

# Import آئزک سیم modules
سے omni.isaac.core import World
سے omni.isaac.core.robots import روبوٹ
سے omni.isaac.core.utils.nucleus import get_assets_root_path
سے omni.isaac.core.utils.prims import create_prim
سے omni.isaac.core.utils.stage import add_reference_to_stage

# Import sensor modules
سے omni.isaac.sensor import RotatingLidarPhysX, Camera
سے omni.isaac.core.sensors import Imu


class AdvancedSensorProcessor:
    """Class کو handle advanced sensor processing"""
    
    def __init__(self):
        self.world = None
        self.روبوٹ = None
        self.lidar = None
        self.camera = None
        self.imu = None
        
        # Sensor data storage
        self.lidar_data = None
        self.camera_data = None
        self.imu_data = None
        
        # روبوٹ state tracking
        self.position_history = deque(maxlen=100)
        self.orientation_history = deque(maxlen=100)
        
        # Mapping اور localization
        self.occupancy_grid = np.zeros((100, 100))  # Simple 100x100 grid
        self.grid_resolution = 0.1  # 10cm per cell
        self.grid_offset = np.array([50, 50])  # مرکز کا grid پر (0,0)
        
        # Feature detection
        self.features = []  # Detected features میں کا/کی ماحول
    
    def setup_environment(self):
        """Set اوپر کا/کی ماحول کے لیے advanced sensor processing"""
        
        # Create کا/کی world
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        
        # Add various objects کو create ایک rich ماحول کے لیے sensing
        # Add multiple obstacles کا different shapes
        create_prim(
            prim_path="/World/Obstacle_Sphere",
            prim_type="Sphere",
            position=[2.0, 1.0, 0.5],
            scale=[0.3, 0.3, 0.3]
        )
        
        create_prim(
            prim_path="/World/Obstacle_Box",
            prim_type="Cube",
            position=[-1.5, 2.0, 0.5],
            scale=[0.5, 0.8, 1.0]
        )
        
        create_prim(
            prim_path="/World/Obstacle_Cylinder",
            prim_type="Cylinder",
            position=[1.0, -2.0, 0.7],
            scale=[0.4, 0.4, 1.4]
        )
        
        # Add کچھ walls
        create_prim(
            prim_path="/World/Wall_Vertical",
            prim_type="Cuboid",
            position=[3.0, 0, 1.0],
            scale=[0.1, 4.0, 2.0]
        )
        
        create_prim(
            prim_path="/World/Wall_Horizontal",
            prim_type="Cuboid",
            position=[0, 3.0, 1.0],
            scale=[4.0, 0.1, 2.0]
        )
        
        # Add روبوٹ
        assets_root_path = get_assets_root_path()
        اگر assets_root_path:
            try:
                self.روبوٹ = self.world.scene.add(
                    روبوٹ(
                        prim_path="/World/AdvancedSensorRobot",
                        name="advanced_sensor_robot",
                        usd_path=assets_root_path + "/Isaac/Robots/TurtleBot3Burger/turtlebot3_burger.usd",
                        position=[0, 0, 0.1],
                        orientation=[0, 0, 0, 1]
                    )
                )
            except:
                # Fallback: create basic روبوٹ
                create_prim(
                    prim_path="/World/AdvancedSensorRobot",
                    prim_type="Xform",
                    position=[0.0, 0.0, 0.5]
                )
        
        # Add sensors کو روبوٹ
        try:
            # Add LiDAR
            self.lidar = RotatingLidarPhysX(
                prim_path="/World/AdvancedSensorRobot/Lidar",
                translation=np.array([0.05, 0, 0.2]),
                config="Example_Rotary",
                depth_range=10.0,
                horizontal_resolution=0.25,
                vertical_resolution=0.5,
                horizontal_fov=360,
                vertical_fov=30,
                rotation_frequency=10,
                samples_per_scan=1440
            )
            self.lidar.initialize()
        except:
            print("LiDAR initialization failed")
        
        try:
            # Add camera
            self.camera = Camera(
                prim_path="/World/AdvancedSensorRobot/Camera",
                position=np.array([0.1, 0, 0.1]),
                frequency=30,
                resolution=(640, 480)
            )
            self.camera.initialize()
        except:
            print("Camera initialization failed")
        
        try:
            # Add IMU
            self.imu = Imu(
                prim_path="/World/AdvancedSensorRobot/Imu",
                position=np.array([0.0, 0.0, 0.1]),
                orientation=np.array([0, 0, 0, 1])
            )
            self.imu.initialize()
        except:
            print("IMU initialization failed")
        
        # Reset کا/کی world
        self.world.reset()
    
    def get_robot_pose(self):
        """Get روبوٹ's current position اور orientation"""
        اگر self.روبوٹ:
            pos, quat = self.روبوٹ.get_world_pose()
            
            # Convert quaternion کو yaw angle
            w, x, y, z = quat
            yaw = math.atan2(2*(w*z + x*y), w*w + x*x - y*y - z*z)
            
            return np.array([pos[0], pos[1], pos[2]]), yaw
        
        return np.array([0.0, 0.0, 0.0]), 0.0
    
    def process_lidar_data(self):
        """Process LiDAR data کو detect obstacles اور map ماحول"""
        اگر نہیں self.lidar:
            return
        
        try:
            lidar_data = self.lidar.get_linear_depth_data()
            
            اگر lidar_data ہے نہیں None اور len(lidar_data) > 0:
                # Get روبوٹ's current pose
                robot_pos, robot_yaw = self.get_robot_pose()
                
                # Convert polar coordinates کو Cartesian کے لیے mapping
                angles = np.linspace(-np.pi, np.pi, len(lidar_data))
                
                # Filter out invalid measurements
                valid_indices = np.کہاں((lidar_data > 0.1) & (lidar_data < 10.0))
                valid_ranges = lidar_data[valid_indices]
                valid_angles = angles[valid_indices]
                
                # Convert کو global coordinates
                local_x = valid_ranges * np.cos(valid_angles)
                local_y = valid_ranges * np.sin(valid_angles)
                
                # Transform کو global frame
                cos_yaw = np.cos(robot_yaw)
                sin_yaw = np.sin(robot_yaw)
                
                global_x = local_x * cos_yaw - local_y * sin_yaw + robot_pos[0]
                global_y = local_x * sin_yaw + local_y * cos_yaw + robot_pos[1]
                
                # Update occupancy grid
                کے لیے x, y میں zip(global_x, global_y):
                    # Convert world coordinates کو grid coordinates
                    grid_x = int((x / self.grid_resolution) + self.grid_offset[0])
                    grid_y = int((y / self.grid_resolution) + self.grid_offset[1])
                    
                    # Check bounds
                    اگر 0 <= grid_x < self.occupancy_grid.shape[0] اور 0 <= grid_y < self.occupancy_grid.shape[1]:
                        # Mark کے طور پر occupied
                        self.occupancy_grid[grid_x, grid_y] = 1
                
                # Also mark روبوٹ's current position کے طور پر free
                robot_grid_x = int((robot_pos[0] / self.grid_resolution) + self.grid_offset[0])
                robot_grid_y = int((robot_pos[1] / self.grid_resolution) + self.grid_offset[1])
                
                اگر 0 <= robot_grid_x < self.occupancy_grid.shape[0] اور 0 <= robot_grid_y < self.occupancy_grid.shape[1]:
                    # Mark روبوٹ position کے طور پر free
                    self.occupancy_grid[robot_grid_x, robot_grid_y] = 0.5  # Free space
                
                print(f"✓ Processed LiDAR data: {len(valid_ranges)} valid measurements")
                return True
            
        except Exception کے طور پر e:
            print(f"✗ LiDAR processing error: {e}")
        
        return False
    
    def process_camera_data(self):
        """Process camera data کو detect features"""
        اگر نہیں self.camera:
            return
        
        try:
            camera_data = self.camera.get_rgb()
            
            اگر camera_data ہے نہیں None اور camera_data.size > 0:
                # میں ایک real نفاذ, آپ کرے گا perform computer vision processing
                # such کے طور پر feature detection, object recognition, etc.
                
                print(f"✓ Processed camera data: shape {camera_data.shape}")
                
                # کے لیے یہ ورک ایکس, we'll just record وہ we رکھتے ہیں image data
                self.camera_data = camera_data
                
                # مثال feature detection (simplified)
                height, width = camera_data.shape[:2]
                
                # Detect corners using ایک simple method (simplified)
                corners = []
                
                # میں ایک real نفاذ, آپ کرے گا use OpenCV یا other CV library
                # کے لیے اب, just simulate detecting کچھ features
                corners.append((width//4, height//4))    # اوپر بائیں
                corners.append((3*width//4, height//4))  # اوپر صحیح
                corners.append((width//4, 3*height//4)) # نیچے بائیں
                corners.append((3*width//4, 3*height//4)) # نیچے صحیح
                
                print(f"✓ Detected {len(corners)} features میں camera view")
                
                return True
        
        except Exception کے طور پر e:
            print(f"✗ Camera processing error: {e}")
        
        return False
    
    def process_imu_data(self):
        """Process IMU data کے لیے state estimation"""
        اگر نہیں self.imu:
            return
        
        try:
            linear_acc = self.imu.get_linear_acceleration()
            angular_vel = self.imu.get_angular_velocity()
            
            # Store data
            self.imu_data = {
                'linear_acceleration': linear_acc,
                'angular_velocity': angular_vel
            }
            
            print(f"✓ Processed IMU data: acc=({linear_acc[0]:.3f}, {linear_acc[1]:.3f}, {linear_acc[2]:.3f})")
            return True
            
        except Exception کے طور پر e:
            print(f"✗ IMU processing error: {e}")
        
        return False
    
    def create_occupancy_map(self):
        """Create اور visualize occupancy map سے sensor data"""
        
        # میں ایک real نفاذ, آپ کرے گا create ایک مزید sophisticated map
        # کے لیے یہ ورک ایکس, we'll just print statistics about کا/کی grid
        
        occupied_cells = np.sum(self.occupancy_grid == 1)
        free_cells = np.sum(self.occupancy_grid == 0.5)
        unknown_cells = np.sum(self.occupancy_grid == 0)
        
        total_cells = self.occupancy_grid.size
        
        print(f"Occupancy map statistics:")
        print(f"  Total cells: {total_cells}")
        print(f"  Occupied: {occupied_cells} ({occupied_cells/total_cells*100:.1f}%)")
        print(f"  Free: {free_cells} ({free_cells/total_cells*100:.1f}%)")
        print(f"  Unknown: {unknown_cells} ({unknown_cells/total_cells*100:.1f}%)")
        
        # میں ایک real نفاذ, آپ کرے گا save یہ کے طور پر ایک image یا use یہ کے لیے navigation
        return self.occupancy_grid
    
    def estimate_robot_motion(self):
        """Estimate روبوٹ motion using sensor fusion"""
        
        # Get current pose
        current_pos, current_yaw = self.get_robot_pose()
        
        # Store میں history
        self.position_history.append(current_pos.copy())
        self.orientation_history.append(current_yaw)
        
        # Estimate velocity اگر we رکھتے ہیں enough history
        اگر len(self.position_history) >= 2:
            # Calculate displacement
            prev_pos = self.position_history[-2]
            displacement = current_pos - prev_pos
            
            # Calculate velocity
            velocity = displacement / 0.01  # یہ ہے approximate
            
            print(f"Estimated velocity: ({velocity[0]:.3f}, {velocity[1]:.3f}, {velocity[2]:.3f}) m/s")
        
        return current_pos, current_yaw
    
    def run_sensor_processing_loop(self, num_steps=1000):
        """Run کا/کی advanced sensor processing loop"""
        print("Starting advanced sensor processing loop...")
        
        کے لیے step میں range(num_steps):
            # Step کا/کی سمولیشن
            self.world.step(render=True)
            
            # Process sensors every few steps
            اگر step % 10 == 0:
                print(f"\nProcessing step {step}...")
                
                # Process تمام sensors
                lidar_ok = self.process_lidar_data()
                camera_ok = self.process_camera_data()
                imu_ok = self.process_imu_data()
                
                # Estimate motion
                pos, yaw = self.estimate_robot_motion()
                print, yaw={yaw:.3f}")
                
                # Print processing status
                status = "✓" اگر else "✗"
                print(f"{status} Sensor processing completed")
            
            # Print occupancy map statistics periodically
            اگر step % 100 == 0 اور step > 0:
                print(f"\nStep {step} - Map Update:")
                self.create_occupancy_map()
        
        print("✓ Advanced sensor processing loop completed")
    
    def run_exercise(self):
        """Run کا/کی complete advanced sensor processing ورک ایکس"""
        print("="*60)
        print("="*60)
        
        try:
            self.setup_environment()
            self.run_sensor_processing_loop(num_steps=1000)
            
            # Final map
            print(f"\nFinal occupancy map:")
            occupancy_map = self.create_occupancy_map()
            
            # Cleanup
            self.world.clear()
            
            
        except Exception کے طور پر e:
            import traceback
            traceback.print_exc()


def main():
    """Main function کو run advanced sensor processing ورک ایکس"""
    sensor_processor = AdvancedSensorProcessor()
    sensor_processor.run_exercise()


اگر __name__ == "__main__":
    main()
```
## vr ک var ک ک s 4: آئزک saum sasidna ش n ش n vucilahumn ٹ

###
mau یک ک saum آئزک ssaum atsasusausa کے کے
Create کا/کی extension script (`~/isaac_sim_exercises/scripts/custom_extension_exercise.py`):
```python
# مثال extension structure
# کے لیے یہ ورک ایکس, we'll show کیسے کو structure اور develop ایک extension

"""
Custom آئزک سیم Extension کے لیے روبوٹکس

یہ مثال shows کیسے کو create ایک custom extension وہ adds روبوٹکس-specific
functionality کو آئزک سیم, such کے طور پر ایک custom UI panel کے لیے روبوٹ control یا
ایک automated testing ڈھانچہ.
"""

import omni.ext
import omni.ui کے طور پر ui
سے omni.isaac.core import World
سے omni.isaac.core.robots import روبوٹ
سے omni.isaac.core.utils.nucleus import get_assets_root_path
import carb

# کا/کی extension class
class RoboticsExtension(omni.ext.IExt):
    """Custom روبوٹکس Extension کے لیے آئزک سیم"""
    
    def on_startup(self, ext_id):
        """Called کب کا/کی extension ہے started"""
        
        # Create ایک UI window کے لیے کا/کی extension
        self._window = ui.Window
        
        کے ساتھ self._window.frame:
            کے ساتھ ui.VStack():
                ui.Label
                
                # Add UI elements کے لیے روبوٹ control
                self._robot_control_frame = ui.CollapsableFrame
                کے ساتھ self._robot_control_frame:
                    کے ساتھ ui.VStack():
                        # Add buttons کے لیے controlling ایک روبوٹ
                        ui.Button("Move Forward", clicked_fn=self._move_forward)
                        ui.Button("Move Backward", clicked_fn=self._move_backward)
                        ui.Button
                        ui.Button
                        ui.Button("Stop", clicked_fn=self._stop_robot)
                
                # Add UI elements کے لیے sensor visualization
                self._sensor_frame = ui.CollapsableFrame("Sensor Visualization")
                کے ساتھ self._sensor_frame:
                    کے ساتھ ui.VStack():
                        ui.Button("Visualize LiDAR", clicked_fn=self._visualize_lidar)
                        ui.Button("Show Camera Feed", clicked_fn=self._show_camera)
        
        # Initialize آئزک سیم components
        self._world = World(stage_units_in_meters=1.0)
        self._robot = None
        
    
    def on_shutdown(self):
        """Called کب کا/کی extension ہے shut نیچے"""
        
        # صاف اوپر UI
        اگر self._window:
            self._window.destroy()
        
        # صاف اوپر آئزک سیم components
        اگر self._world:
            self._world.cleanup()
    
    def _move_forward(self):
        """Move روبوٹ forward"""
        # میں real نفاذ, یہ کرے گا control کا/کی روبوٹ
        
    def _move_backward(self):
        """Move روبوٹ backward"""
        
    def _turn_left(self):
        """Turn روبوٹ بائیں"""
        
    def _turn_right(self):
        """Turn روبوٹ صحیح"""
        
    def _stop_robot(self):
        """Stop روبوٹ motion"""
    
    def _visualize_lidar(self):
        """Visualize LiDAR data"""
    
    def _show_camera(self):
        """Show camera feed"""


# Additional utility functions کے لیے کا/کی extension
class RobotController:
    """Utility class کے لیے روبوٹ control"""
    
    def __init__(self, world):
        self._world = world
        self._current_robot = None
    
    def add_robot(self, robot_usd_path, position=[0, 0, 0]):
        """Add ایک روبوٹ کو کا/کی سمولیشن"""
        try:
            روبوٹ = self._world.scene.add(
                روبوٹ(
                    prim_path="/World/CustomRobot",
                    name="custom_robot",
                    usd_path=robot_usd_path,
                    position=position
                )
            )
            self._current_robot = روبوٹ
            return روبوٹ
        except Exception کے طور پر e:
            carb.log_error
            return None
    
    def control_robot(self, linear_vel, angular_vel):
        """Control کا/کی روبوٹ using differential drive"""
        اگر self._current_robot:
            # میں ایک real نفاذ, یہ کرے گا set کا/کی روبوٹ's wheel velocities
            # کے لیے اب, we'll just log کا/کی کمانڈ


def create_extension():
    """Helper function کو create کا/کی extension"""
    return RoboticsExtension()
```
## vr ک ias 5: چ LAN ے کی کی کی کی کی کی

###
Create ایک script کو run تمام exercises (`~/isaac_sim_exercises/scripts/run_all_exercises.py`):
```python
#!/usr/bin/env python3
"""
Script کو run تمام آئزک سیم exercises
"""

import subprocess
import sys
import os

def run_exercise(script_path, description):
    """Run ایک single ورک ایکس"""
    print(f"\n{'='*60}")
    print(f"Running: {description}")
    print(f"Script: {script_path}")
    print(f"{'='*60}")
    
    try:
        result = subprocess.run([sys.executable, script_path], 
                              capture_output=True, text=True, timeout=120)
        
        اگر result.returncode == 0:
            اگر result.stdout:
                print(f"Output:\n{result.stdout[-1000:]}")  # Last 1000 chars
        else:
            اگر result.stderr:
                print(f"Error:\n{result.stderr}")
    
    except subprocess.TimeoutExpired:
        print")
    except Exception کے طور پر e:

def main():
    """Run تمام exercises"""
    print("="*60)
    
    exercises_dir = os.path.dirname(os.path.abspath(__file__))
    
    exercises = [
    ]
    
    کے لیے script, description میں exercises:
        script_path = os.path.join(exercises_dir, script)
        اگر os.path.exists(script_path):
            run_exercise(script_path, description)
        else:
    
    print(f"\n{'='*60}")
    print("="*60)

اگر __name__ == "__main__":
    main()
```
## خ LAA صہ

عمال اعمت وِسر ِ ِ ِ ڈھ ی ی ی ی ی ی ی ی ی ڈھ

1.
2.
3
4.
5
6.

یہ ماؤس ہ atauch ss ے taurbi ف ​​raaum iSrat ی ہیں۔ ہیں۔ ک a/کی maus ظ aa ہ r arta ی ہیں:

- کی s ے ک s پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی
- کی s ے ک v v v mamatl ف ss ی nssr ز ک v mrbwau ک r یں
- کی s ے ک ک ک ve ve n ف sowauchn awrss ک نہرول لاؤچورٹیم
- کی s ے ک s iaun ی mri ضی کے maubi یک sasaun ش in ش jahar ک ri ک l کے l یے

گئی ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف نیچے ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف ف. اوسو الکحمنیی ک ک خ خ خ خ خ خ خ ط ط ط vr پ r پ ur پ ud mautawaun ، ssunssr jusur فی our فی کی کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کی کی کی کی کے کے کے کے کے کے کے کے کے کے کے کے کے کے کی کی کی کی کے کے کے کے کے کے کے کے کے کے کے کی کی کی کی کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے

یہ ک/کی ہف ہف آئزک آئزک آئزک آئزک ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک ڈی ڈی ڈی آئزک آئزک آئزک آئزک آئزک آئزک آئزک ڈی ڈی ڈی ڈی آئزک آئزک آئزک آئزک آئزک آئزک آئزک ڈی ڈی ڈی آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک ڈی آئزک ڈی آئزک ڈی آئزک آئزک آئزک آئزک آئزک آئزک آئزک ڈی ڈی ڈی ڈی آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک ڈی ڈی ڈی آئزک آئزک آئزک آئزک آئزک آئزک آئزک ڈی ڈی آئزک ڈی آئزک ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی نیچے ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی ڈی Mauml -irata ہے۔
