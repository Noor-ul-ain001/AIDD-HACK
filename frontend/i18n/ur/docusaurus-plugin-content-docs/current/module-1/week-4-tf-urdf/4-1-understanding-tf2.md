---


sidebar_position: 1
difficulty: beginner


---
# 4.1: ROS 2 ٹی ایف 2 (aransasaurm llaiaubrachra) ک سوسمونا

## ج a ج

یہ ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- SSMAU کہ
- s یکھیں کی s ے ٹی ٹی یف یف یف 2 Vr ک s maa ئی rrsos 2
- ٹی ٹی ِ s 2 ک a ک amaal ک j ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ
۔
- - لالگا ٹی یف یف 2 maamalی rewbobo ، منشرن
- کامن ٹی ٹی ِ ِs 2 عمور ک بی گ بی ک ک r یں

## ماؤم ی ttaulahatat

### vausr ڈی n یٹ ف raum

کے کے کے کے کے j ہ ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک
- **Base frame** (`base_link`, `base_footprint`): روبوٹ's مرکز
- **Sensor frames** (`camera_frame`, `laser_frame`): Sensor mounting positions
- **World/mapping frame** (`map`, `odom`): Reference کے لیے position tracking
- **اختتام-effector frame** (`gripper`, `tool0`): روبوٹ's working point
### کی vi ٹی ٹی 2 ہے کی کی ض ض ض ض ض ض ض ہے

ک ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ
- ڈیٹ asas matali maumatli ssasnsr
- ٹ r یک ک a/کی پ پ پ waun ک a bubbob کے حصے حصے کے کے کے کے کے
- ک vaur ڈی n یٹ ssusum کے Mababauna ٹ n ٹ s ک v juaaa ئ na ٹ s ک v atbadacl athr یں
- مستقل مقامی تفہیم کو برقرار رکھیں

## ٹی ایس 2

###

1
2
3
4. **ٹی ایف 2 Message Types**: `tf2_msgs/TFMessage` اور related types
### ٹی ٹی ا (2 ببماؤبلہ ٹی ایف

ٹی ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ ٹ نیچے ٹ ٹ ٹ -
- جبر کارالدیہ
- بدی api
- سپورٹ کے Lیے Lیے کسٹم ڈیٹا ٹائپ
- ک l ی nr la یح d گی - خ خ خ خ خ خ

## نعریت کی تبدالی

### جامد تبدالی

ایبدالی ہیں ف ف کے کے کے کے کے کے کے کے کے جی جی جی جی جی جی جی جی جی جی جی جی جی

** اذر نعقہ: **
```python
import rclpy
سے rclpy.نود import نود
سے tf2_ros import StaticTransformBroadcaster
سے geometry_msgs.msg import TransformStamped
import tf_transformations

class StaticFramePublisher:
    def __init__(self):
        super().__init__('static_frame_publisher')
        
        # Create ایک StaticTransformBroadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish کا/کی static transform
        self.publish_static_transform()
    
    def publish_static_transform(self):
        # Create ایک transform
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().اب().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'
        
        # Set translation (x, y, z)
        t.transform.translation.x = 0.1  # 10 cm forward
        t.transform.translation.y = 0.0  # نہیں lateral offset
        t.transform.translation.z = 0.2  # 20 cm اوپر
        
        # Set rotation
        # کے لیے ایک simple 90-degree rotation around Z axis
        q = tf_transformations.quaternion_from_euler(0, 0, 1.5708)  # 90 degrees میں radians
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Send کا/کی transform
        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info('Published static transform: base_link -> laser_frame')

def main(args=None):
    rclpy.init(args=args)
    نود = StaticFramePublisher()
    rclpy.spin
    نود.destroy_node()
    rclpy.shutdown()
```
### مت ح آر ٹی جیٹیل ی

منتھر ویست (جی si ے ، ، ، ، ، ، ، ، ، ، ، ، گھ گھ گھ گھ گھ گھ گھ گھ Sunass گھ Sunasr ، ، ، ، ، ، ، ، ، ، ، ، ، ، ula ssansr ،۔

** اذر نعقہ: **
```python
import rclpy
سے rclpy.نود import نود
سے tf2_ros import TransformBroadcaster
سے geometry_msgs.msg import TransformStamped
import tf_transformations
import math

class DynamicFramePublisher:
    def __init__(self):
        super().__init__('dynamic_frame_publisher')
        
        # Create ایک TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create ایک timer کو periodically broadcast transforms
        self.timer = self.create_timer(0.1, self.broadcast_transform)  # 10 Hz
        self.time_step = 0.0
    
    def broadcast_transform(self):
        # Create ایک transform
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().اب().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'rotating_sensor'
        
        # Make کا/کی transform oscillate کے لیے demonstration
        # Translation moves میں ایک circle
        radius = 0.3  # 30 cm radius
        t.transform.translation.x = radius * math.cos(self.time_step)
        t.transform.translation.y = radius * math.sin(self.time_step)
        t.transform.translation.z = 0.5  # Fixed height
        
        # Rotation changes کے اوپر time
        q = tf_transformations.quaternion_from_euler(0, 0, self.time_step)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Send کا/کی transform
        self.tf_broadcaster.sendTransform(t)
        
        self.time_step += 0.1  # Increment کے لیے next iteration
        
        اگر self.time_step % 10 < 0.1:  # Log every 10 seconds
            self.get_logger().info(f'Published dynamic transform: base_link -> rotating_sensor')

def main(args=None):
    rclpy.init(args=args)
    نود = DynamicFramePublisher()
    rclpy.spin
    نود.destroy_node()
    rclpy.shutdown()
```
## سنسن ے v v ٹ Raynsurum ز

### bin ی ad ی ٹ rensaurm snna ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے

** اذر نعقہ: **
```python
import rclpy
سے rclpy.نود import نود
سے tf2_ros import TransformListener, Buffer
سے geometry_msgs.msg import PointStamped

class FrameListener:
    def __init__(self):
        super().__init__('frame_listener')
        
        # Create ایک transform buffer
        self.tf_buffer = Buffer()
        
        # Create ایک transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create ایک timer کو periodically look اوپر transforms
        self.timer = self.create_timer(1.0, self.lookup_transform)
    
    def lookup_transform(self):
        try:
            # Look اوپر کا/کی transform سے 'base_link' کو 'laser_frame'
            اب = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link',  # Target frame
                'laser_frame',  # Source frame
                اب)  # Time (use 'اب' کے لیے زیادہ تر recent)
            
            # Log کا/کی transform
            self.get_logger().info(
                f'Translation: x={trans.transform.translation.x:.3f}, '
                f'y={trans.transform.translation.y:.3f}, '
                f'z={trans.transform.translation.z:.3f}')
            self.get_logger().info(
                f'Rotation: x={trans.transform.rotation.x:.3f}, '
                f'y={trans.transform.rotation.y:.3f}, '
                f'z={trans.transform.rotation.z:.3f}, '
                f'w={trans.transform.rotation.w:.3f}')
        
        except Exception کے طور پر e:
            self.get_logger().error}')

def main(args=None):
    rclpy.init(args=args)
    نود = FrameListener()
    rclpy.spin
    نود.destroy_node()
    rclpy.shutdown()
```
###

** اذر نعقہ: **
```python
import rclpy
سے rclpy.نود import نود
سے tf2_ros import TransformListener, Buffer
سے geometry_msgs.msg import PointStamped, TransformStamped
سے builtin_interfaces.msg import Time
import time

class TimedFrameListener:
    def __init__(self):
        super().__init__('timed_frame_listener')
        
        # Create ایک transform buffer
        self.tf_buffer = Buffer()
        
        # Create ایک transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create ایک پبلشر کے لیے transformed points
        self.point_pub = self.create_publisher(PointStamped, 'transformed_point', 10)
        
        # Timer کو demonstrate timed transforms
        self.timer = self.create_timer(2.0, self.lookup_timed_transform)
    
    def lookup_timed_transform(self):
        try:
            # Create ایک point میں کا/کی laser frame
            point_in_laser = PointStamped()
            point_in_laser.header.frame_id = 'laser_frame'
            point_in_laser.header.stamp = self.get_clock().اب().to_msg()
            point_in_laser.point.x = 1.0  # 1 meter میں سامنے کا laser
            point_in_laser.point.y = 0.0
            point_in_laser.point.z = 0.0
            
            # Transform کا/کی point کو base_link frame
            point_in_base = self.tf_buffer.transform(
                point_in_laser,    # Input point
                'base_link',       # Target frame
                timeout=rclpy.duration.Duration(seconds=1.0))  # Timeout
            
            # Log کا/کی result
            self.get_logger().info(
                f'Transformed point: ({point_in_base.point.x:.3f}, '
                f'{point_in_base.point.y:.3f}, {point_in_base.point.z:.3f}) '
                f'میں base_link frame')
            
            # Publish کا/کی transformed point
            self.point_pub.publish(point_in_base)
        
        except Exception کے طور پر e:
            self.get_logger().error(f'Transform failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    نود = TimedFrameListener()
    rclpy.spin
    نود.destroy_node()
    rclpy.shutdown()
```
## ڈیٹ a -ک s ک s s se

### تبد ی ک ک ے ے vaaun ٹ s ، waur ز ، owaor پ vat

** اذر نعقہ: **
```python
import rclpy
سے rclpy.نود import نود
سے tf2_ros import TransformListener, Buffer
سے geometry_msgs.msg import PointStamped, Vector3Stamped, PoseStamped
سے sensor_msgs.msg import LaserScan

class DataTransformer:
    def __init__(self):
        super().__init__('data_transformer')
        
        # Create ایک transform buffer
        self.tf_buffer = Buffer()
        
        # Create ایک transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe کو laser scan
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # پبلشر کے لیے transformed data
        self.transformed_scan_pub = self.create_publisher(
            LaserScan, 'scan_in_base_link', 10)
    
    def scan_callback(self, msg):
        try:
            # Transform کا/کی entire scan کو base_link frame
            # یہ ہے ایک simplified مثال - میں practice, آپ'd transform individual points
            transform = self.tf_buffer.lookup_transform(
                'base_link',      # Target frame
                msg.header.frame_id,  # Source frame (usually laser_frame)
                msg.header.stamp,     # Time کا کا/کی scan
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Log وہ we رکھتے ہیں کا/کی transform
            self.get_logger().info(
                f'Got transform سے {msg.header.frame_id} کو base_link')
        
        except Exception کے طور پر e:
            self.get_logger().error}')

def main(args=None):
    rclpy.init(args=args)
    نود = DataTransformer()
    rclpy.spin
    نود.destroy_node()
    rclpy.shutdown()
```
## ٹی ia یف 2 MAUC C ++

###
```cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ٹی ایف 2/LinearMath/Quaternion.h>

class StaticFramePublisher : public rclcpp::نود
{
public:
    StaticFramePublisher() : نود("static_frame_publisher")
    {
        tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>;
        
        // Publish کا/کی static transform
        publish_static_transform();
    }

private:
    void publish_static_transform()
    {
        geometry_msgs::msg::TransformStamped t;
        
        t.header.stamp = یہ->get_clock()->اب();
        t.header.frame_id = "base_link";
        t.child_frame_id = "laser_frame";
        
        // Set translation
        t.transform.translation.x = 0.1;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.2;
        
        // Set rotation (90 degrees around Z axis)
        ٹی ایف 2::Quaternion q;
        q.setRPY(0, 0, 1.5708);  // roll, pitch, yaw
        
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        tf_publisher_->sendTransform(t);
        RCLCPP_INFO, "Published static transform: base_link -> laser_frame");
    }
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticFramePublisher>());
    rclcpp::shutdown();
    return 0;
}
```
```cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class FrameListener : public rclcpp::نود
{
public:
    FrameListener() : نود("frame_listener")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create ایک timer کو periodically look اوپر transforms
        timer_ = یہ->create_wall_timer(
            std::chrono::seconds(1),
            std::bind);
    }

private:
    void lookup_transform()
    {
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "base_link",    // Target frame
                "laser_frame",  // Source frame
                ٹی ایف 2::TimePointZero);  // زیادہ تر recent available
            
            RCLCPP_INFO,
                "Translation: x=%f, y=%f, z=%f",
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z);
        }
        catch {
            RCLCPP_WARN, "کر سکتا تھا نہیں get transform: %s", ex.کیا());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcl::TimerBase::SharedPtr timer_;
};
```
## ٹی ia یف 2 Biqutri ی n ط r یقہ ک ک ar

###
- Use consistent naming: `sensor_name_frame` (e.g., `camera_frame`, `lidar_frame`)
- Use semantic names: `map`, `odom`, `base_link`, `base_footprint`
- REP-105 ک NON ش N ز فالو کالو فالو احریں کے L یے Maesahari ف Ri ی M ہnaam

### ک ارورڈ گی کے taaatat
```python
# Efficient transform lookup
class EfficientTransformer:
    def __init__(self):
        super().__init__('efficient_transformer')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Cache transforms کے لیے frequently accessed frames
        self.cached_transforms = {}
    
    def get_cached_transform(self, target_frame, source_frame):
        cache_key = f"{target_frame}_{source_frame}"
        
        # Try کو get سے cache پہلا
        اگر cache_key میں self.cached_transforms:
            return self.cached_transforms[cache_key]
        
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            # Cache کا/کی transform
            self.cached_transforms[cache_key] = transform
            return transform
        except Exception کے طور پر e:
            self.get_logger().error(f'Transform lookup failed: {e}')
            return None
```
```python
سے tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class RobustTransformer:
    def __init__(self):
        super().__init__('robust_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def safe_lookup_transform(self, target_frame, source_frame, time=None):
        try:
            اگر time ہے None:
                time = rclpy.time.Time()
            
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, time)
            return transform, True  # Success
        except LookupException:
            self.get_logger().warn
            return None, False
        except ConnectivityException:
            self.get_logger().warn
            return None, False
        except ExtrapolationException:
            self.get_logger().warn
            return None, False
        except Exception کے طور پر e:
            self.get_logger().error
            return None, False
```
## ٹی ٹی ٹی ٹی

### ک اِنسان ڈ- لالقان
```bash
# View کا/کی TF tree
ros2 run tf2_tools view_frames

# Echo ایک specific transform
ros2 run tf2_ros tf2_echo base_link laser_frame

# View TF tree کے طور پر image (creates frames.pdf)
# بعد running view_frames
evince frames.pdf  # یا آپ کا سسٹم's PDF viewer
```
### ٹی ا (2 ماناگر
```bash
# Monitor TF quality
ros2 run tf2_ros tf2_monitor
```
## عام ٹی ٹی ِ ِ ِs 2 اوسو اواور اائل

### 1
```python
# Problem: Getting "کر سکتا تھا نہیں find transform" errors
# Solution: Wait کے لیے transforms کو ہونا available

import time
سے rclpy.duration import Duration

class WaitForTransform:
    def __init__(self):
        super().__init__('wait_for_transform')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def lookup_with_wait(self):
        try:
            # Wait کے لیے transform کو become available
            self.tf_buffer.wait_for_transform(
                'base_link', 'laser_frame', 
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
            
            # اب کرنا کا/کی lookup
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'laser_frame', rclpy.time.Time())
            
            return transform
        except Exception کے طور پر e:
            self.get_logger().error
            return None
```
### 2۔ واٹ کے مسعسل
```python
# Problem: Transform پر غلط time
# Solution: Use appropriate time synchronization

class TimeSyncedTransformer:
    def __init__(self):
        super().__init__('time_synced_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def transform_with_sensor_time(self, sensor_msg):
        try:
            # Use کا/کی time سے کا/کی sensor message کے لیے transform
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                sensor_msg.header.frame_id,
                sensor_msg.header.stamp,
                timeout=Duration(seconds=0.1))
            
            return transform
        except Exception کے طور پر e:
            self.get_logger().error(f'Time-synced transform failed: {e}')
            return None
```
## خ LAA صہ

یہ الل l ی ں

- ف raumwau ک v مربو ک r یں
- جامد اواورس متالرک تبدالیو کی نعرقت
- تبدالیو ں a atamal ک samal ہ swaus snn ک o snnr ہ a ہے
- ک vaur ڈی n یٹ ف raumwiu کے Mababan ڈیٹ a ک a ک s jo ک se
- بہترین طریقہ کار اور common debugging techniques

ٹی ک ک کی کی کی ک ک ک ک ک ک ک ک ک ک ک ک کی ک کی ک ک ک ک ک ک کی کی کی کی کی ک ک ک ک ک ک کی کی کی کی کی کی ک ک ک کی کی کی کی کی کی کی کی کی ک کی کی کی کی کی کی کی کی ک کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی ک a/کی کی کی کی a/کی کی کی کی a/کی کی کی کی کی کی کی کی a/. ا کے کے کے ، ، ، ک ک ک a/کی ، ، ہ ہM ی ی ی ی ی ڈی یunaیفaaئیڈ rewubobw کی tacl کی stl) کی stlaid sataid satais atatis atata saatis atati sata sata sata sata sata sata sata sata sata sata sata گے ک ک ک ک ک ک ٹی ٹی 2 ک 2 ک ک Vubui ک ک
