---


sidebar_position: 1
difficulty: intermediate


---
# ساسا 2: اسسع ق revs کی bin ی aad ی buat یں

## ج a ج

یہ ایٹ نے اسحاق روس کو متعارف کرایا ، نویڈیا کے روبوسس پلیٹ فارم وِی نے کa/کی روبوبی ٹ jrobouch jriauchnگ ssaum (ros) کے saatiھ nvidia کی gpu-accelerated llaiubrauchwi ں کے کے یے یے یے ذہ ذہ ذہ ذہ ذہ ذہ ذہ ذہ ذہ ذہ ذہ کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے

## ss یکھ n ے کے maua ص d

کے ذ ک ک ک ک a/کی کی خ خ خ خ atatam ک a یہ یہ ہف ہف آپ آپ ک ک ک ک ک ک ک ک ے ے ے ے گ گ گ گ گ
- اماو کہ
- s یٹ یٹ یٹ یٹ یٹ یٹ کے کے کے کے ک ک ک ک ک ے ے rewbo ٹک ے ے ے کیش کیش کیش کیش کیش
- آئز aa ک ک r oas ک as ک a ک amal ک satamal ہ samal ہ Celau ئے پ aa ث laiuchnwauch فذ ک s
- یsaق ق کے کے کے saa ؤٹی suaumoud ہ آ r osr oass 2 ssaum usso murbouc

## تعارف اوسو اوساق روس

آئز aa ک rws ہے nwaua ک a j j jrutahar ataaur asswr an ی sowaun asas ٹیک bul ٹ کے s آ s آ s آ s آ یہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ نیچے ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ

###

1.
2.
3.
4

### آئز aa ک aa rews آ r کیٹیکچ r کیٹیکچ r
```
ہارڈ ویئر Layer (NVIDIA Jetson/RTX)
        |
GPU Acceleration Layer
        |
Isaac ROS Packages
        |
ROS 2 Middleware
        |
Applications
```
## یsaحa آ آ r oss ک s ک s ک so onsasacasal arna

### ی ssm کی ض srvrauatat

| a جز aa ء | S M SS ے ک M | ttحق ک rd ہ |
| ----------- | --------- | ----------------- |
| GPU | nvidia gpu کے SAAT ھ CUDA | nvidia rtx 3060 ی a b ہ atr |
| CUDA | 11.8+ | 12.0+ |
| OS | اوبنٹو 22.04 | اوبنٹو 22.04 lts |
| رام | 8 جی B ی | 16+ جی B ی |
| ی سورس | 10 جی B ی | 20+ جی B ی |

### tn ی b ِ n ط r یقہ 1: ڈی b ی n پیکیجز
```bash
# Add Isaac ROS repository
sudo apt update && sudo apt install سافٹ ویئر-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://repos.packages.nvidia.com/keys/تمام-keys.gpg | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-archive-keyring.gpg
echo "deb [signed-کے ذریعے=/usr/share/keyrings/nvidia-archive-keyring.gpg] https://repos.packages.nvidia.com/isaac/$(lsb_release -cs)/ تمام main" | sudo tee /etc/apt/sources.list.d/nvidia-isaac-repos-$(lsb_release -cs).list
sudo apt update

# Install Isaac ROS packages
sudo apt install nvidia-isaac-ros-gem
```
### تنی باب باب ط r یقہ 2: ڈ vaur
```bash
# Pull Isaac ROS Docker image
docker pull nvcr.io/nvidia/isaac-ros:latest

# Run Isaac ROS container
docker run --gpus تمام -یہ --rm --net=host nvcr.io/nvidia/isaac-ros:latest
```
## vor یsح (rws پیکیجز

### آئز aa Revs aa پ raulaul گ

اوسور تتومنہ ک a پ پ ہ ہ پ ک ک ک ک ک ک ک ک ک ک ک ک ک ک یگ یگ یگ
```python
import rclpy
سے rclpy.نود import نود
سے sensor_msgs.msg import Image
سے isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

class AprilTagDetector:
    def __init__(self):
        super().__init__('apriltag_detector')
        
        # Subscribe کو camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # پبلشر کے لیے detections
        self.detection_pub = self.create_publisher(
            AprilTagDetectionArray,
            '/apriltag_detections',
            10
        )
    
    def image_callback(self, msg):
        # Process image کے لیے AprilTag detection
        pass
```
### یsaحa ق revs Baِsrی slium

رِکل البعری سلیم کے لوکل الو الاعد
```python
import rclpy
سے rclpy.نود import نود
سے sensor_msgs.msg import Image, CameraInfo
سے geometry_msgs.msg import PoseStamped
سے nav_msgs.msg import Odometry

class VisualSLAMNode:
    def __init__(self):
        super().__init__('visual_slam')
        
        # Subscribe کو stereo camera یا RGB-D data
        self.left_image_sub = self.create_subscription(
            Image,
            '/camera/بائیں/image_raw',
            self.left_image_callback,
            10
        )
        
        self.right_image_sub = self.create_subscription(
            Image,
            '/camera/صحیح/image_raw',
            self.right_image_callback,
            10
        )
        
        # Publishers کے لیے pose اور map
        self.pose_pub = self.create_publisher(PoseStamped, '/visual_slam/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/visual_slam/odometry', 10)
```
## آئز aa ک ک ک ک ک ک ن ن چ ہی

### maa ئی llan چ ف aa ئ l
```python
سے launch import LaunchDescription
سے launch_ros.ایکشنز import ComposableNodeContainer
سے launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Isaac ROS AprilTag container
    apriltag_container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        پیکیج='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                پیکیج='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag',
                parameters=[{
                    'family': 'tag36h11',
                    'max_tags': 10,
                    'tag_size': 0.166
                }]
            )
        ],
        output='screen',
    )

    return LaunchDescription([apriltag_container])
```
## یsح (آ آ کے کے کے کے خی خی خی خی کی کی کی ala ئپ laiauchn ز

### تاؤسور ی پ rrususasna گ پ aa ئپ llaiun
```python
# Isaac ROS provides optimized image preprocessing نوڈز
سے launch import LaunchDescription
سے launch_ros.ایکشنز import ComposableNodeContainer
سے launch_ros.descriptions import ComposableNode

def generate_image_preprocessing_pipeline():
    image_processing_container = ComposableNodeContainer(
        name='image_processing_container',
        namespace='',
        پیکیج='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Image resize نود
            ComposableNode(
                پیکیج='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                name='resize_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image', 'camera/image_raw'),
                    ('camera_info', 'camera/camera_info'),
                    ('resized/image', 'camera/image_resized'),
                    ('resized/camera_info', 'camera/camera_info_resized'),
                ],
            ),
            
            # Image format converter
            ComposableNode(
                پیکیج='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::FormatConverterNode',
                name='format_converter_node',
                remappings=[
                    ('image', 'camera/image_resized'),
                    ('camera_info', 'camera/camera_info_resized'),
                    ('output/image', 'camera/image_formatted'),
                    ('output/camera_info', 'camera/camera_info_formatted'),
                ],
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([image_processing_container])
```
## اعامام کے ssaٹ ھ ھ ھ revs 2 maawawlautی niھam

## پھr

کے کے ی ی ss maausoula ی ati ی nھam:
```python
# Standard ROS 2 نود consuming Isaac ROS output
import rclpy
سے rclpy.نود import نود
سے geometry_msgs.msg import PoseStamped
سے sensor_msgs.msg import Image

class IsaacIntegrationNode:
    def __init__(self):
        super().__init__('isaac_integration_node')
        
        # Subscribe کو Isaac ROS pose output
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/pose',
            self.pose_callback,
            10
        )
        
        # Subscribe کو Isaac ROS processed image
        self.processed_image_sub = self.create_subscription(
            Image,
            '/isaac_ros/image_processed',
            self.image_callback,
            10
        )
    
    def pose_callback(self, msg):
        # Use Isaac ROS pose میں standard ROS 2 ایپلی کیشن
        self.get_logger().info(f'Position: x={msg.pose.position.x}, y={msg.pose.position.y}')
    
    def image_callback(self, msg):
        # Process Isaac ROS accelerated image
        self.get_logger().info(f'Received processed image: {msg.width}x{msg.height}')
```
## عمالہ وورک اِس

ہف ہف t ہ 's vr ک ک ss ss ss mausr assr aِs آئزک آئزک آئزک ک کal aaal aail aaaul aaauc laiauchan atrataub idauna شaml ہے:

1. یsaق ق ق پ آپ ک ک ک ک ڈ Oaulam
2. امری ہ nn ن شکی شکی شکی شکی شکی شکی l کے l یے ISAAC ROS پ RWSASSN گ
3. امت الکلی ٹگ ک at at ll گ an ے aa ئپ llaiaun ک o jna فذ ک ri یں
4.

## خ LAA صہ

ہف اوس نِس اوسحا vs ک ک ک J ک ک ک ک ک ک ک nvidia ک کaidia کadia یزadia یزaidarauraus jrautahr jraubws پilیٹ ف پ ف ف ف ف ll کے prespecide nevigation. آپ '' n ے t ، ک ک ک ک ، ، ، ، ، ، کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے ی کے ی کے کے کے کے کے کے کے کے کے کے کے ، ، ، ، ، ، ،
