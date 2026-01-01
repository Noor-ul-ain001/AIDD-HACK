---


sidebar_position: 3
difficulty: intermediate


---
# 3.3: آئزک سیسم بنیدی بائیو اواور روبوسس تحقورات

## ج a ج

یہ ا کی ص ص ص ص ص ص ص ص ص کی کی کی کی ک ک ک ک ک ک ک ، ، ، ، گے گے ، ، ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- mamr ط r یقے ss ے ک a/کی آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک کی
- اواور تشکیل دیں روبو ماڈلز کی تشکیل کریں
- owr ک v mautl ف ssunssr کی کی sasam کی کی کی کی کی کی کی کی کی
- طbیsaیat کی خص swaut ک sw ssm جھیں vors mwad
- کام کے سعتا ی ی پ پ پ پ پ پ پ ، ، پ ، ، ، ، پ پ پ پ پ پ پ ، پ ، ، ، پ پ پ پ پ پ پ پ پ پ
- سیسس اوسر بنییدی روبوبو احنورول اواور سوسوولن وور بلکہ
- ک-/کی کی کے drmaun آئزک SSASM OSSM OSSR Revs 2 کے DRMACAN TALAUCHAT ASS

## آئزک ساسم اوانگرفیس اواور نِن

### اعم اعنی ی یس اعیس

ک a/کی آئزک آئزک آئزک sims ana ٹ r فی ss maus کئی ک ک ک l ی d ی aa جز a ء ش aaml ہیں:

1
2.
3
4
5
6

### bin ی aad ی ni ی sowautn ک nahrul ز

- ** مدار **: Alt + baaئیں ماؤس بٹن
- ** پین **: ALT + صحیح ماؤس بٹن
.
- ** فوکس پr انتخاب **: f کلید
- ** ف ra ی m Jamamam **: یک کی کی

### یs ٹیج ٹ r ی Jn ی Owaun

کA/کی اسٹیج ٹری ٹری ڈسپلے کرتا ہے کہ تa/کی PIXAR USD نام دینے والے کنونشن کا استعمال کرتے ہوئے JAM عناصر maus/USD USD کا منظر:

- **/World**: Root کا کا/کی سمولیشن stage
- **/دنیا/روبوٹ **: روبو ماڈل
- **/ورلڈ/مااؤل **: جامد ماؤل عناصر
- **/دنیا/لائٹس **: روشنی کے ذرائع
- **/دنیا/کیمرے **: کیمرا عناصر

## USD تِٹورات اواور پ راؤوم ز

###

USD (آف a قی منشر کی تال) ہے ک ک ک ک a/کی aiaiun ڈیش n ڈیش n ک a آئزک saum:

.
.
- ** تعلقات **: پرائمز کے مابین رابطے
- **Variants**: Different configurations کا کا/کی same prim
.

### اوس یsso ووسس توڑ پ raaum ز بونانا
```python
# مثال کا working کے ساتھ prims میں آئزک سیم
سے omni.isaac.core import World
سے omni.isaac.core.utils.stage import add_reference_to_stage
سے omni.isaac.core.utils.prims import get_prim_at_path
سے omni.isaac.core.utils.stage import get_stage_units
سے pxr import UsdGeom, Gf

# Create ایک world
world = World(stage_units_in_meters=1.0)

# Add ایک cube prim کو کا/کی stage
def add_cube_to_stage():
    """Add ایک cube کو کا/کی stage using USD primitives"""
    سے omni.isaac.core.utils.prims import create_prim
    سے omni.isaac.core.utils.nucleus import get_assets_root_path
    
    # Create ایک cube primitive
    cube_path = "/World/Cube"
    create_prim(
        prim_path=cube_path,
        prim_type="Cube",
        position=[0, 0, 1.0],
        orientation=[0, 0, 0, 1],  # [x, y, z, w] quaternion
        scale=[1, 1, 1]
    )
    
    # Access کا/کی prim اور modify اس کا properties
    cube_prim = get_prim_at_path(cube_path)
    اگر cube_prim:
        print(f"Prim type: {cube_prim.GetTypeName()}")

# Call کا/کی function
add_cube_to_stage()
```
## روبو مامالات لِلیق وور امتورکنی

### آ سان روبو مامال الالالہ

mau آئزک ssam ، rrobouc maaul ہیں aaam ط vr پ sr USD USD USD ف aaulvi ک a ک asatamail asrati ے juc ہ sotli خ a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a گی a ہ a ہ a گی a گی a گی a گی a یہ a ں ک کی s ے ک s ک ک یقی یقی یقی ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ڈ ڈ

#### USD Rrobou ف a ئ l ک a ڈھ an چہ
```usd
# مثال USD file کے لیے ایک simple روبوٹ (روبوٹ.usd)
#usda 1.0

def Xform "روبوٹ" (
    prepend apiSchemas = ["IsaacArticulatedRobot"]
)
{
    def Xform "base_link" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {
        def Cube "visual" (
            prepend apiSchemas = ["ColliderAPI"]
        )
        {
            size 1
            color (0.8, 0.8, 0.8)
        }

        def Sphere "left_wheel" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI"]
        )
        {
            # Wheel properties
        }

        def Sphere "right_wheel" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI"]
        )
        {
            # Wheel properties
        }
    }
}
```
#### روبوبو ماممم آئزک ساسم احستا جن بننانا
```python
# Script کو create ایک simple روبوٹ میں آئزک سیم
سے omni.isaac.core import World
سے omni.isaac.core.robots import روبوٹ
سے omni.isaac.core.utils.stage import add_reference_to_stage
سے omni.isaac.core.utils.nucleus import get_assets_root_path
سے omni.isaac.core.utils.prims import create_prim
import numpy کے طور پر np

class SimpleDiffDriveRobot:
    """Simple differential drive روبوٹ نفاذ"""
    
    def __init__(
        self,
        prim_path: str,
        name: str = "simple_diff_drive_robot",
        usd_path: str = None,
        position: np.ndarray = np.array([0, 0, 0]),
        orientation: np.ndarray = np.array([0, 0, 0, 1]),
    ) -> None:
        """Initialize کا/کی differential drive روبوٹ"""
        super().__init__(
            prim_path=prim_path,
            name=name,
            usd_path=usd_path,
            position=position,
            orientation=orientation,
        )

def setup_robot_in_world():
    """Set اوپر کا/کی روبوٹ میں کا/کی world"""
    # Create کا/کی world
    world = World(stage_units_in_meters=1.0)
    
    # Create کا/کی روبوٹ
    try:
        # Try کو use ایک default روبوٹ asset
        assets_root_path = get_assets_root_path()
        اگر assets_root_path ہے None:
            print("کر سکتا تھا نہیں find آئزک سیم assets path")
            return None
            
        # مثال: Create ایک روبوٹ سے ایک default asset
        robot_path = assets_root_path + "/Isaac/Robots/TurtleBot3Burger/turtlebot3_burger.usd"
        
        # Add کا/کی روبوٹ کو کا/کی world
        روبوٹ = world.scene.add(
            روبوٹ(
                prim_path="/World/روبوٹ",
                name="turtlebot",
                usd_path=robot_path,
                position=[0, 0, 0.1],
                orientation=[0, 0, 0, 1]
            )
        )
        
        return روبوٹ
        
    except Exception کے طور پر e:
        
        # Fallback: Create ایک simple روبوٹ using basic prims
        create_prim(
            prim_path="/World/روبوٹ",
            prim_type="Xform",
            position=[0, 0, 0.5]
        )
        
        # Create ایک simple base
        create_prim(
            prim_path="/World/روبوٹ/Base",
            prim_type="Cube",
            position=[0, 0, 0],
            scale=[0.5, 0.5, 0.1]
        )
        
        # Create wheels
        create_prim(
            prim_path="/World/روبوٹ/LeftWheel",
            prim_type="Cylinder",
            position=[0, 0.3, 0],
            scale=[0.15, 0.15, 0.05]
        )
        
        create_prim(
            prim_path="/World/روبوٹ/RightWheel",
            prim_type="Cylinder",
            position=[0, -0.3, 0],
            scale=[0.15, 0.15, 0.05]
        )
        

# Execute ترتیب
روبوٹ = setup_robot_in_world()
```
## ssunssr an ض maam آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک

### ssunssr ش aaml ک aml۔ روبو

آئزک saum n ے ssunsr کی mautli ف sasam کی ح maa ی t کی کی

#####
```python
# مثال کا adding ایک camera کو ایک روبوٹ
سے omni.isaac.sensor import Camera
import numpy کے طور پر np

def add_camera_to_robot(robot_prim_path):
    """Add ایک camera sensor کو کا/کی روبوٹ"""
    # Create camera پر ایک position relative کو کا/کی روبوٹ
    camera = Camera(
        prim_path=f"{robot_prim_path}/front_camera",
        position=np.array([0.2, 0, 0.1]),
        frequency=30,
        resolution=(640, 480)
    )
    
    # Initialize کا/کی sensor
    camera.initialize()
    
    # مثال کا getting data سے کا/کی camera
    def capture_image():
        try:
            rgb_image = camera.get_rgb()
            depth_image = camera.get_depth()
            print(f"Captured RGB image: shape={rgb_image.shape}")
            return rgb_image, depth_image
        except Exception کے طور پر e:
            print(f"Error capturing image: {e}")
            return None, None
    
    return camera, capture_image

# Usage
camera, capture_func = add_camera_to_robot
```
#####
```python
# مثال کا adding ایک LiDAR کو ایک روبوٹ
سے omni.isaac.sensor import LidarRtx
import numpy کے طور پر np

def add_lidar_to_robot(robot_prim_path):
    """Add ایک LiDAR sensor کو کا/کی روبوٹ"""
    # Create LiDAR sensor
    lidar = LidarRtx(
        prim_path=f"{robot_prim_path}/lidar",
        translation=np.array([0.1, 0, 0.2]),
        orientation=np.array([0, 0, 0, 1]),
        config="Example_Rotary",
        depth_range=10.0,
        horizontal_resolution=0.25,
        vertical_resolution=0.5,
        horizontal_fov=360,
        vertical_fov=30,
        rotation_frequency=10,
        samples_per_scan=1440,
        phase_offset=0
    )
    
    lidar.initialize()
    
    # مثال کا getting LiDAR data
    def get_lidar_scan():
        try:
            data = lidar.get_linear_depth_data()
            print(f"LiDAR data shape: {data.shape}")
            return data
        except Exception کے طور پر e:
            print(f"Error getting LiDAR data: {e}")
            return None
    
    return lidar, get_lidar_scan

# Usage
lidar, lidar_func = add_lidar_to_robot
```
#### عمو سنسر
```python
# مثال کا adding ایک IMU کو ایک روبوٹ
سے omni.isaac.core import World
سے omni.isaac.core.sensors import Imu
import numpy کے طور پر np

def add_imu_to_robot(robot_prim_path):
    """Add ایک IMU sensor کو کا/کی روبوٹ"""
    # Add IMU کو کا/کی روبوٹ's base link
    imu = Imu(
        prim_path=f"{robot_prim_path}/imu",
        position=np.array([0.0, 0.0, 0.05]),
        orientation=np.array([0, 0, 0, 1])
    )
    
    # Initialize کا/کی IMU
    imu.initialize()
    
    # مثال کا getting IMU data
    def get_imu_data():
        try:
            linear_acceleration = imu.get_linear_acceleration()
            angular_velocity = imu.get_angular_velocity()
            print(f"IMU Linear Acc: {linear_acceleration}")
            print(f"IMU Angular Vel: {angular_velocity}")
            return linear_acceleration, angular_velocity
        except Exception کے طور پر e:
            print(f"Error getting IMU data: {e}")
            return None, None
    
    return imu, get_imu_data

# Usage
imu, imu_func = add_imu_to_robot
```
## طbیsaesahat تال

### ط b ی aa ی at کی خص oautaat

اعبعیت کی خص خص خص oautat
```python
# مثال کا configuring physics properties
سے omni.isaac.core.utils.prims import setRigidBodyProperties
سے omni.isaac.core.utils.prims import setStaticColliderProperties
سے pxr import Gf

def configure_robot_physics(robot_prim_path):
    """Configure physics properties کے لیے کا/کی روبوٹ"""
    # Set rigid body properties کے لیے کا/کی روبوٹ base
    setRigidBodyProperties(
        prim_path=f"{robot_prim_path}/Base",
        mass=10.0,  # kg
        linear_damping=0.05,
        angular_damping=0.1,
        max_linear_velocity=1000.0,
        max_angular_velocity=1000.0,
        enable_gyroscopic_forces=True
    )
    
    # Set collision properties
    setStaticColliderProperties(
        prim_path=f"{robot_prim_path}/Base",
        approximation_shape="mesh",  # یا "convexHull", "boundingCube", etc.
        restitution=0.1,  # Bounciness (0-1)
        friction=0.5,     # Friction coefficient
        static_friction=0.5
    )
    
    # Configure wheel physics کے لیے differential drive
    کے لیے wheel_name میں ["LeftWheel", "RightWheel"]:
        wheel_path = f"{robot_prim_path}/{wheel_name}"
        setRigidBodyProperties(
            prim_path=wheel_path,
            mass=1.0,
            linear_damping=0.1,
            angular_damping=0.2
        )
        
        setStaticColliderProperties(
            prim_path=wheel_path,
            restitution=0.1,
            friction=0.8,  # Higher friction کے لیے traction
            static_friction=0.8
        )
        

# Apply physics تشکیل
configure_robot_physics
```
## ros 2 an ض ممام

### رعببس - سرما آئزک سوس ک ایسس روس 2

ک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک نیچے آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک ی ی آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک ی ی ی آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک
```python
# مثال کا setting اوپر ROS 2 integration
سے omni.isaac.core.utils.extensions import enable_extension
import rclpy
سے geometry_msgs.msg import Twist
سے sensor_msgs.msg import LaserScan, Image, Imu
import numpy کے طور پر np

def setup_ros2_bridge():
    """ترتیب ROS 2 bridge میں آئزک سیم"""
    # Enable کا/کی ROS 2 bridge extension
    enable_extension("omni.isaac.ros2_bridge.humble")
    
    # Initialize ROS 2
    rclpy.init()
    
    # Create ROS 2 نود
    نود = rclpy.create_node('isaac_sim_bridge')
    
    # Create publishers کے لیے sensor data
    lidar_pub = نود.create_publisher(LaserScan, '/scan', 10)
    imu_pub = نود.create_publisher(Imu, '/imu/data', 10)
    image_pub = نود.create_publisher(Image, '/camera/image_raw', 10)
    
    # Create سبسکرائیبر کے لیے روبوٹ control
    def cmd_vel_callback(msg):
        """Handle velocity commands سے ROS 2"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # میں ایک real نفاذ, یہ کرے گا control کا/کی روبوٹ
        # یہ ہے کہاں آپ'd update کا/کی روبوٹ's wheel velocities
        control_robot(linear_x, angular_z)
    
    cmd_vel_sub = نود.create_subscription(Twist, '/cmd_vel', cmd_vel_callback, 10)
    
    def publish_sensor_data():
        """Publish sensor data کو ROS 2 ٹاپکس"""
        # یہ کرے گا ہونا called periodically کے دوران سمولیشن
        
        # مثال: publish fake lidar data
        lidar_msg = LaserScan()
        lidar_msg.header.stamp = نود.get_clock().اب().to_msg()
        lidar_msg.header.frame_id = 'lidar_frame'
        lidar_msg.angle_min = -np.pi
        lidar_msg.angle_max = np.pi
        lidar_msg.angle_increment = 0.01
        lidar_msg.range_min = 0.1
        lidar_msg.range_max = 10.0
        lidar_msg.ranges = [1.0] * 628  # مثال: 628 readings
        
        lidar_pub.publish(lidar_msg)
    
    return نود, publish_sensor_data

def control_robot(linear_vel, angular_vel):
    """Control کا/کی روبوٹ based پر velocity commands"""
    # میں ایک real نفاذ, یہ کرے گا control کا/کی روبوٹ's actuators
    # کے لیے differential drive: convert linear/angular velocities کو wheel velocities
    # left_wheel_vel = (linear_vel - angular_vel * wheel_base / 2) / wheel_radius
    # right_wheel_vel = (linear_vel + angular_vel * wheel_base / 2) / wheel_radius
    pass

# ترتیب ROS 2 bridge
نود, publisher_func = setup_ros2_bridge()
```
## موود اواور شیڈ شیڈ شیڈ شیڈ شیڈ

### ک aam asrn ے ے ے ے کے ssaaut mwad

آئزک ssaum nvidia کے mdl (maad ی tare یف کی ز bain bain) ک ایک سسٹامال کی ہے۔
```python
# مثال کا applying materials کو objects
سے omni.isaac.core.utils.materials import create_preview_surface
سے omni.isaac.core.utils.prims import get_prim_at_path
سے pxr import Gf

def apply_materials_to_robot():
    """Apply materials کو روبوٹ components"""
    # Create different materials کے لیے روبوٹ parts
    base_material = create_preview_surface(
        prim_path="/World/Looks/RobotBaseMaterial",
        color=(0.2, 0.6, 0.8),  # Blue color کے لیے base
        metallic=0.1,
        roughness=0.8
    )
    
    wheel_material = create_preview_surface(
        prim_path="/World/Looks/WheelMaterial",
        color=(0.1, 0.1, 0.1),  # Black color کے لیے wheels
        metallic=0.2,
        roughness=0.5
    )
    
    # Apply materials کو روبوٹ parts
    base_prim = get_prim_at_path
    اگر base_prim:
        # Apply material کو کا/کی base prim
        pass  # Material assignment کرے گا ہونا done through USD APIs
    

# Apply materials
apply_materials_to_robot()
```
## ssmwl یش n vr ک ف loc

### ba ی s ک smwoln llous

smaun ے ک a/کی smwl یش n wr ک ف lv maus آئزک Saum:
```python
# مثال کا ایک basic سمولیشن loop
سے omni.isaac.core import World
سے omni.isaac.core.utils.stage import add_reference_to_stage
import numpy کے طور پر np

def run_basic_simulation():
    """Run ایک basic سمولیشن کے ساتھ روبوٹ"""
    # Create world کے ساتھ physics settings
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    
    # Add روبوٹ کو کا/کی world
    assets_root_path = get_assets_root_path()
    اگر assets_root_path:
        روبوٹ = world.scene.add(
            روبوٹ(
                prim_path="/World/روبوٹ",
                name="turtlebot",
                usd_path=assets_root_path + "/Isaac/Robots/TurtleBot3Burger/turtlebot3_burger.usd",
                position=[0, 0, 0.1],
                orientation=[0, 0, 0, 1]
            )
        )
    
    # Reset کا/کی world کو initialize تمام objects
    world.reset()
    
    # Main سمولیشن loop
    کے لیے میں میں range(1000):  # Run کے لیے 1000 steps
        # Step کا/کی physics سمولیشن
        world.step(render=True)
        
        # Get روبوٹ state
        اگر میں % 100 == 0:  # Every 100 steps
            # Get روبوٹ position
            اگر روبوٹ:
                pos, quat = روبوٹ.get_world_pose()
            
            # مثال کا applying force کو روبوٹ
            # روبوٹ.apply_force_at_position(force=np.array([1.0, 0, 0]), position=روبوٹ.get_position())
    
    # Cleanup
    world.clear()

# Run کا/کی basic سمولیشن
# یہ ہے typically run سے within آئزک سیم
```
## آئزک ss ی m asasaun ش ni ز کے کے کے کے کے ِ ِ کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے

### مسعود توسس کے کے کے کے ِ ِ ِ ِ ِ ِ ِ کے کے کے

آئزک SWM آ آ ہے کے کے کے SAAT ھ کئی کئی کئی کئی SASAUNI ش NA ش خ AA ص ط ط ط Vr پ R پ LL یے rewboauss:

1.
2
3
4.
5

### توسسعیسو اِس ایلو احرنا
```python
# مثال کا enabling extensions programmatically
سے omni.isaac.core.utils.extensions import enable_extension
سے omni.isaac.core.utils.extensions import disable_extension

def enable_robotics_extensions():
    """Enable essential روبوٹکس extensions"""
    extensions_to_enable = [
        "omni.isaac.ros2_bridge.humble",  # ROS2 bridge
        "omni.isaac.sensor",              # Sensor extensions
        "omni.isaac.range_sensor",        # Range sensors
        "omni.isaac.actuators",           # Actuator models
        "omni.isaac.occupancy_map"        # Occupancy mapping
    ]
    
    کے لیے ext میں extensions_to_enable:
        try:
            enable_extension(ext)
            print(f"✓ Enabled: {ext}")
        except Exception کے طور پر e:

# Enable extensions
enable_robotics_extensions()
```
## ڈی بون گ اوسو وِسو الیعسن

### ڈی b گ n گ ٹ wl ز آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک

آئزک ساسم نِٹ کئی ٹ ٹ vli ف raaum کیے ہیں ہیں کے l یے l یے l یے ڈیبگنگ:
```python
# مثال کا debugging techniques
سے omni.isaac.core.utils.prims import get_all_matching_child_prims
سے pxr import UsdGeom

def debug_robot_setup():
    """Debug کا/کی روبوٹ ترتیب"""
    # List تمام prims میں کا/کی stage
    prims = get_all_matching_child_prims("/World")
    کے لیے prim میں prims:
        print(f"  - {prim.GetPath().pathString}")
    
    # Check روبوٹ prim properties
    robot_prim = get_prim_at_path
    اگر robot_prim:
        print}")
        
        # Check اگر physics ہے applied
        physics_api = robot_prim.GetAppliedAPIs()
        print(f"Applied APIs: {[api.GetSchemaClassPrimDefinition().GetTypeName() کے لیے api میں physics_api]}")
    
    # Visualize coordinate frames
    # آئزک سیم رکھتا ہے built-میں tools کے لیے visualizing frames
    # (Window -> Visualize -> Show Caches -> Show Xforms)

# Run debug
debug_robot_setup()
```
## خ LAA صہ

یہ الل l ی ی ے ے ک jamamamaul n ے vr ڈ ک a/کی bin ی aad ی torat ک ک آئزک آئزک saum کے ll یے rewbwaus:

- **Interface navigation**: Understanding کا/کی main components اور controls
۔
- **روبوٹ model creation**: Creating اور configuring روبوٹ models
.
.
۔
.
- **سمولیشن workflows**: Understanding کا/کی سمولیشن loop
.
.

یہ a/کی ف aiaun ڈیش n ڈیش n ڈیش ض ض ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ض ہ ض ض ض ض ہ ہ ض ض ہ ض ض ض ض ض ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ کے کے کے کے کے ہ ہ ہ ہ ہ M AA گ LA/ذی ، ہ ہ ہ M AMAAL یہ MAUCHOI ں JALYUS ک V TT LAI ک SR یں
