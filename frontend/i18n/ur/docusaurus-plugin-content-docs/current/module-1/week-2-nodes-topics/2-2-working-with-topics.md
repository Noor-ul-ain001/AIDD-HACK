---


sidebar_position: 2
difficulty: intermediate


---
# 2.2: کام اعسرا کے ساسا ھ آروس روس 2 ٹ a پک s

## ج a ج

یہ ذیلی ماڈیول explores ROS 2 ٹاپکس میں detail, جس ہیں کا/کی primary mechanism کے لیے asynchronous مواصلات between نوڈز using ایک publish-subscribe pattern.

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- - سمہ/کی ش aa ئ ae ک sr یں
- پblی ش ش ز ز کے کے کے کے ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص ص
- isam کے SAAT ھ MAUTLI پیغ AMAT کی کی کی SASAAM OSOSWR ک SAUM ک SAUM ک AAMAT AAMAT
- maiur کی tacl ar یں - srws (qos) کی پ ala ی sasa ی کے l ِ ll کے a ٹ a پک sas
- ROS 2 ولی ک JASTAMAL ک J ہ OI ئے AB گ JOWRS MANAHR ٹ AASS

## ٹ aa پک mwa ص Lat nmun ہ

ROS 2 asatamal asrata ہے ** ش aa ئ ae jais-sbasaishraiauchb icri یں ** maa ڈ l کے ll یے ہ ہm آہnگی mwaaulat. maaؤs یہ پیٹ پیٹ پیٹ

- ** پبلشرز ** پیغامات بھیجیں
- ** صارفین ** پیغامات وصول کرتے ہیں
- متعدعد البالشرز اواورس کے صارسن
- مواصلات ہے decoupled میں time اور space

### bin ی aad ی bi ہ a ؤ
```
پبلشر نود → ٹاپک → سبسکرائیبر نود
     ↓                    ↑
پبلشر نود →--------→ سبسکرائیبر نود
```
## پ بلیئر ز بونا

### ام
```python
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
        msg.data = f'Hello World: {self.میں}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.میں += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```
### c ++ پ bl ش r
```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::نود
{
public:
    MinimalPublisher() : نود("minimal_publisher")
    {
        publisher_ = یہ->create_publisher<std_msgs::msg::String>;
        timer_ = یہ->create_wall_timer(
            500ms, std::bind);
        count_ = 0;
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello World: " + std::to_string(count_++);
        RCLCPP_INFO, "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::پبلشر<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};
```
## ص ari فی n کی تال

### r s s s یs bi یs یs آ آ r آ ئی biی biی آ آ
```python
import rclpy
سے rclpy.نود import نود
سے std_msgs.msg import String

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
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
```
### c ++ ssasbsraaئbr
```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MinimalSubscriber : public rclpp::نود
{
public:
    MinimalSubscriber() : نود("minimal_subscriber")
    {
        subscription_ = یہ->create_subscription<std_msgs::msg::String>(
            "ٹاپک", 10,
            std::bind);
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO, "میں heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
```
## واول ٹی - سروس (Qos) کی Jatratauatbat

Qos پ Alasasa ی a ں ک na ٹ rlol کی s ے پیغ amat p naaa ش ri ی n کے mababidun ف raaium ک readi ہ owr ص ari فی n کے Mababachn:
```python
سے rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Create ایک custom QoS profile
qos_profile = QoSProfile(
    depth=10,  # Number کا messages کو buffer
    reliability=ReliabilityPolicy.RELIABLE,  # یا BEST_EFFORT
    history=HistoryPolicy.KEEP_LAST,  # یا KEEP_ALL
    durability=DurabilityPolicy.VOLATILE  # یا TRANSIENT_LOCAL
)

# Use کا/کی custom QoS profile
پبلشر = نود.create_publisher
subscription = نود.create_subscription
```
### اعسم اوتستبات
- **Reliability**: `RELIABLE` ensures تمام messages ہیں delivered; `BEST_EFFORT` doesn't guarantee delivery
- **History**: `KEEP_LAST` keeps کا/کی زیادہ تر recent N messages; `KEEP_ALL` keeps تمام messages
- **Durability**: `VOLATILE` کے لیے temporary messages; `TRANSIENT_LOCAL` کے لیے persistent messages
## اعنی ی maubi پیغ پیغ کی کی کی کی کی کی کی کی کی کی کی کی پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ پیغ کی کی کی کی کی کی کی پیغ پیغ پیغ پیغ کی کی کی

### اعنی ی ی مسٹر ضی کے MaUab ق پیغ امات بونا

1.
2. Define آپ کا message میں ایک `.msg` file:
```
# MyCustomMessage.msg
string name
int32 id
float64 value
bool active
```
3. اپ ڈیٹ ڈیٹ ڈیٹ آپ.
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
4. اپ ڈیٹ ڈیٹ ڈیٹ cmakelists.txt:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyCustomMessage.msg"
)
```
## پھr
```python
سے my_package_msgs.msg import MyCustomMessage

# پبلشر کے ساتھ custom message
پبلشر = نود.create_publisher(MyCustomMessage, 'custom_topic', 10)

msg = MyCustomMessage()
msg.name = 'Robot1'
msg.id = 1
msg.value = 3.14
msg.active = True

پبلشر.publish(msg)

# سبسکرائیبر کے ساتھ custom message
subscription = نود.create_subscription(
    MyCustomMessage,
    'custom_topic',
    callback,
    10
)
```
## ٹ a پک ٹ ٹ ٹ vli آیات manaprn گ

### ک اِنسان ڈ- لالقان
```bash
# List تمام ٹاپکس
ros2 ٹاپک list

# Get information about ایک ٹاپک
ros2 ٹاپک info /topic_name

# Echo messages سے ایک ٹاپک
ros2 ٹاپک echo /topic_name

# Publish کو ایک ٹاپک سے کمانڈ line
ros2 ٹاپک pub /topic_name std_msgs/msg/String "data: 'Hello'"

# Print کا/کی type کا ایک ٹاپک
ros2 ٹاپک type /topic_name

# Show bandwidth usage کا ٹاپکس
ros2 ٹاپک bw

# Show Hz rate کا ایک ٹاپک
ros2 ٹاپک hz /topic_name

# Show latency کا ایک ٹاپک
ros2 ٹاپک delay /topic_name
```
### اعلی دِل ما ی معادنہ
```bash
# Echo کے ساتھ field filtering
ros2 ٹاپک echo /topic_name --field data

# Echo کے ساتھ limiting messages
ros2 ٹاپک echo /topic_name --field data --count 5

# Echo کے ساتھ specific format
ros2 ٹاپک echo /topic_name --field data --field_filter "data"
```
## ٹ a پک ڈی b گ n گ

### ماؤتری کہ مسا ئ ث ث

1.
   - چیک ک r یں
   - Qus Qos maubaut کی t ص d یق ک ri یں پ ila گ r asusr ss ی bsraausbr کے drmauna
   - ک v یقی n ی Buna ئیں کہ noc ہیں ہیں چ چ چ چ چ چ چ چ

2.
   - Qos ہ sur ی پ ala ی si ی ی ک ک ک ک ک ک ک ک ک ک
   - گہ raa ئی کی Jenasasb کی mnasasb کی taid یق ک ri یں

3.
   - یٹ یٹ vr ک ک na یکٹ vi چیک ک ک ک ک ک ک ک ک ک ک ک ک ک
   - Qos wausna یی taa کی tartautautbat کی ص ص ص ص ص ک ک ک ک ک ک ک ک ک
   - سیسم وسائل کی نگرانی کریں

### ڈی b گ n گ گ w ڈ maamal
```python
def debug_topic_info:
    # Get list کا ٹاپکس اور ان کا types
    topic_names_and_types = نود.get_topic_names_and_types()
    
    کے لیے name, types میں topic_names_and_types:
        
        # Get specific info about کا/کی ٹاپک
        try:
            info = نود.get_publishers_info_by_topic(name)
            print(f"  Publishers: {len(info)}")
            کے لیے pub میں info:
                print(f"    - {pub.node_name}, QoS: {pub.qos_profile}")
                
            info = نود.get_subscriptions_info_by_topic(name)
            print(f"  Subscribers: {len(info)}")
        except Exception کے طور پر e:
            print(f"  Error getting info: {e}")
```
## babٹbr ی ط r یقہ ک ari
1.
2.
3.
4.
5.

## خ LAA صہ

یہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ج ج ہ ہ ہ ہ ہ ہ ہ ج ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ذی ذی ذی ذی ذی ذی ذی ذی ذی ذی ذی ذی ذی ذی ہ ذی ذی ذی ذی ہ ِ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ
