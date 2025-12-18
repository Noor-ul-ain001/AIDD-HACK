---


sidebar_position: 2
difficulty: beginner


---
# سیس 2: نوزی اوس

## ج a ج

ہف ہف t ہ n ے bun ی aad ی taivrat ک v ف sos کی a ہے۔ نوس ہیں ک ک ک a/کی bun ی ad ی Amladriudid auna ٹ s ک a a (rws 2 پ rworamam ، ٹ پک پک پک پک ہیں s ہیں کے ک ک ک کی کی کی کی کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے پک کے پک کے پک ہیں کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے.

## ss یکھ n ے کے maua ص d

کے ذ ک ک ک ک a/کی کی خ خ خ خ atatam ک a یہ یہ ہف ہف آپ آپ ک ک ک ک ک ک ک ک ے ے ے ے گ گ گ گ گ
- شmw کہ/کی ص ص Jauch Maus ros ros ros ros ros 2
- اورس رن روس 2 نوز بنائیں
- ٹ a پک- پ-پ r mubn ی muwaulat پ r aml ک r یں
- پblassriز اوسور صارفین ک ک v mamr ط r یقے ss ے esataamal alsr یں
- ڈی b گ نوس Mwavalat کے maaamalat

## nwwoss mus revs 2

یک یک ک ک ک ک ک ک ک ک ک ک ک ک V ہ V ہ ہ ہ آئی آئی آئی آئی آئی آئی آئی آئی آئی آئی آئی ط ط ط ط ط ط ی ی ک ک ک ک ک ک ک ہ ہ ہ ہ ہ ہ پ پ پ پ پ کے پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ ک ک ہ ی ی ی ک ک ک ک ک ک کے ط ی ی ک ک ک ک ک ک کے کے کے ط ی ی

###

یہ a ں 's ک a/کی bun ی ad ی Jun چہ ک a iair ws 2 no:
```python
import rclpy
سے rclpy.نود import نود

class MyNode:
    def __init__(self):
        super().__init__('node_name')
        # نود initialization code goes یہاں

def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    
    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_node.destroy_node()
        rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## ٹ AASS OWR پیغ AMAT

ٹ A پک S ہیں Namaud Busouc کے کے کے ج ج ج ج J Nn Nn SAS کے کے J کے T AT AT AT AMAT۔ عمات ہیں اعدعد وحر کے ڈھ ڈھ ڈھ ڈھ j j j j j j j j j j j j کے کے کے jn nn nn ros 2 متعاد العدعد العدع ف raum israta ف

###
```python
# پبلشر مثال
import rclpy
سے rclpy.نود import نود
سے std_msgs.msg import String

class MinimalPublisher:
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.میں = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.میں
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.میں += 1

# سبسکرائیبر مثال
class MinimalSubscriber:
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'ٹاپک',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_publisher.destroy_node()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
```
## اعلی دِکورات

### واوال ٹی a ایک ایس آر ڈبلیو ایس (QoS)

ishr oasos 2 maiur-srws (qos) کی tt tt tt tt tt tt tt کی
```python
سے rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# Create ایک custom QoS profile
qos_profile = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE
)

# Use یہ کب creating ایک پبلشر
پبلشر = نود.create_publisher
```
### ٹ a پک ک Man ڈز

موڈ امان ڈ- لال in ٹ ٹ ٹ ٹ کے کے کے کے کے کے کے کے ِ ِ ِ ِ ِ ِ ہیں ہیں ہیں ہیں ہیں ہیں ہیں ستی کے ستی کے ستی ھ aa پک aa پک s:
```bash
# List تمام ٹاپکس
ros2 ٹاپک list

# Echo messages پر ایک ٹاپک
ros2 ٹاپک echo /topic_name

# Print information about ایک ٹاپک
ros2 ٹاپک info /topic_name

# Publish ایک message کو ایک ٹاپک
ros2 ٹاپک pub /topic_name std_msgs/String "data: 'Hello World'"
```
## عمل mamamal: iri جہ ح rarat ک a saunsr

، ، maumle ط vr پ کے کے کے کے کے ح ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک
```python
# temperature_publisher.py
import rclpy
سے rclpy.نود import نود
سے std_msgs.msg import Float32
import random

class TemperaturePublisher:
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = Float32()
        # Simulate temperature reading (20-30 degrees Celsius)
        msg.data = 20.0 + random.uniform(0, 10)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing temperature: {msg.data:.2f}°C')

# temperature_subscriber.py
class TemperatureMonitor:
    def __init__(self):
        super().__init__('temperature_monitor')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10)
        self.subscription  # prevent unused variable warning

    def temperature_callback(self, msg):
        temperature = msg.data
        اگر temperature > 25.0:
            self.get_logger().warn
        else:
            self.get_logger().info(f'Temperature: {temperature:.2f}°C')
```
## ڈی b گ n گ ٹپ s
1. Use `ros2 ٹاپک list` کو verify آپ کا ٹاپکس exist
2. Use `ros2 ٹاپک echo /topic_name` کو see اگر messages ہیں ہوتے ہوئے published
3. Check نود logs کے ساتھ `ros2 run --prefix 'gdb -ex run --args' package_name node_name` کے لیے debugging
4. Use `rqt_graph` کو visualize کا/کی نود مواصلات graph
## ہ ar ڈ vaur tatalat

ک bi ی ک aam aam aarii ے ے کے saesati ی ہ ہ ہ ڈ ڈ ڈ ڈ ڈ ڈ ڈ ڈ ڈ ڈ ڈ ڈ ڈ
- نون کے درمیون نونن وِسرک کے رببطے اِن یقی یقی n ی Buna ئیں
- b ی n ڈ ot ھ کی ح ح ح ح کے کے کے ll یے saunssr ڈیٹ a ar پ r غ vr غ ri ک
- aiaaun ٹ کے کے l یے Llincsی چٹaئی raal یئ aa ئ m ia یپ li کیش n ز
- mnasasb غ غ غ غ ک ک ک کے کے کے کے l ِ ہی ہی ڈ ہی ہی ar ڈ ہی ہی ہی ar ڈ ہی ہی ہی ہی ہی ہی

## خ LAA صہ

ہف ہف ہف ہف ہف ہف ہف ہف ہف nn/ک bin ی AD ی BULAUN گ BULA ک S ک A/ROS ROS ROS ROS ROS ROS ROS 2 MOWAULAT. آپ ساسا کی s ے ک ک v taclیق noc ، پ پ پilشraus پ پ ص ص ص ص ص ص ص ص ص ص ص صiraid ، صiraur کaam کے mautli پیغ پیغam کی پیغam کی کی کی کی کی پیغ پیغ پیغ اگلا ، ہ ہ ہ M srossss ossn ز کے کے کے کے کے کے کے کے کے ِ ِ ِ

## ماؤس

1. تال یق ک r یں nowaus vaus n ے ک a/کی mwaud ہ ssussm isaausm oso ک so asoa کی کی کی کی کی
2. تال شکی ک ک ک s ی bi ھی ش aaadad ہ aaum ٹ a ئ m asaumauss ulaulaulau a a an ک
3. امل - نیس
4.
