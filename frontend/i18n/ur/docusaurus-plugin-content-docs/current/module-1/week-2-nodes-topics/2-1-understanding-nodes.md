---


sidebar_position: 1
difficulty: beginner


---
# 2.1: روس 2 -نوو ک v -smichna

## ج a ج

یہ یہ ک ، ، ، ، J ، J ، J ، J ، J J J J J J j ک ک ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ، ،

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- smausw کہ کہ vos nowd ہے اواور اوس ک اوسس رول مِسوسس روس 2
- Create نوڈز میں both Python اور C++
- مناسب نود لالئف ساسل مینجمنٹ کو نافذ کریں
- نوڈ پیرامیٹرز کو سمجھیں
- Use نود ترکیب کے لیے efficient applications

## کی a ہے ہے ہے ہے ہے نہیں

اِسم آئی ا ی ایس آروس 2 ، اِس ** نوک ** ہے اِس ایس ایس ایس ایس ایس ایس ایس ایس ایس ایس ایس ایس ایس ایس وِنتی اِنتی احام نے idaalی ہے۔ نوس ہیں ہیں ک ک ک a/کی bun ی Adi ی jun ٹ ک a umauchna maus maus ros ros ros ros ros ros ros ros aia aia a گی a گی a گی a گی a ک a ک sa mausollr sausos ی ی auswr aiِvr aِs saesadat bulکہ یسرم السورم کی شک شک شک شک شک شک شک شک شک شک کیش ی ی کیش کیش کیش کیش کیش کیش کیش کیش شک شک

###

1
2.
3
4.

## now ک ک v taclllll ہے ہے ہے

### bin ی aad ی اب ڈ ڈھ an چہ
```python
import rclpy
سے rclpy.نود import نود

class MyNode:
    def __init__(self):
        super().__init__('my_node_name')
        # نود initialization code goes یہاں
        self.get_logger().info

def main(args=None):
    rclpy.init(args=args)
    نود = MyNode()
    rclpy.spin
    نود.destroy_node()
    rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
### nwaڈ نعم احور کی یک یک San ی at

ROS 2 خ ud buad nwr ڈ jur کے ت ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ ذ
```python
# اگر another نود named 'my_node' exists,
# یہ نود کرے گا ہونا renamed کو something like 'my_node_1'
نود = MyNode()  # Uses default name سے class
نود = MyNode(node_name='my_custom_name')  # یا specify custom name
```
## nwaus mah c ++ بونا
```cpp
#include <rclcpp/rclcpp.hpp>

class MyNode : public rclcpp::نود
{
public:
    MyNode() : نود("my_node_name")
    {
        RCLCPP_INFO, "نود initialized");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
```
## نووی پی رامور ز

nwaus ir sauta ہے پی ramamir ز jobol ک بول آر یں ک o ک s
```python
import rclpy
سے rclpy.نود import نود

class ParameterNode:
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters کے ساتھ default values
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('count_threshold', 10)
        self.declare_parameter('frequency', 1.0)
        
        # Get پیرامیٹر values
        self.my_param = self.get_parameter('my_parameter').value
        self.threshold = self.get_parameter('count_threshold').value
        self.freq = self.get_parameter('frequency').value
        
        self.counter = 0

def main(args=None):
    rclpy.init(args=args)
    
    # Pass parameters کب creating نود
    نود = ParameterNode()
    rclpy.spin
    نود.destroy_node()
    rclpy.shutdown()
```
```bash
# کمانڈ line
ros2 run my_package my_node --ros-args -p my_parameter:=new_value -p frequency:=2.0

# میں launch file
سے launch import LaunchDescription
سے launch_ros.ایکشنز import نود

def generate_launch_description():
    return LaunchDescription([
        نود(
            پیکیج='my_package',
            executable='my_node',
            parameters=[
                {'my_parameter': 'new_value'},
                {'frequency': 2.0}
            ]
        )
    ])
```
## نوزی لالئف سیسل مِممنمنی

ros 2 n ے ے ے slaslaslaslaslaslaslaslaslassl ssasm کے کے کے کے کے کے کے کے کے ک ک ک ک ک ک ک ک ک ک ک ک ک
```python
import rclpy
سے rclpy.لائف سائیکل import LifecycleNode, LifecycleState, TransitionCallbackReturn
سے rclpy.لائف سائیکل import نود کے طور پر ROS2Node

class LifecycleManagedNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_node')
        self.get_logger().info

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info
        # ترتیب publishers, subscribers, etc.
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info
        # Activate publishers, subscribers, etc.
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info
        # Deactivate publishers, subscribers, etc.
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info
        # صاف اوپر resources
        return TransitionCallbackReturn.SUCCESS
```
## نود ترکیب

nwausatraub n ے jaidas زی زی زی زی زی زی ہ ہ ju زی jui زی ک v iau ہی ہی ہی ہی ہی ہی ہی ہی ہی کے کے j کے j کے j کے j کے کے کے کے کے چ چ چ چ چ چ چ چ چ کی چ کی کی کی کی کی کی کی
```python
import rclpy
سے rclpy.نود import نود
سے rclpy.executors import SingleThreadedExecutor
سے rclpy.callback_groups import MutuallyExclusiveCallbackGroup
سے std_msgs.msg import String

class PublisherNode:
    def __init__(self):
        super().__init__('publisher_node')
        self.پبلشر = self.create_publisher(String, 'composed_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Composed message {self.counter}'
        self.پبلشر.publish(msg)
        self.counter += 1

class SubscriberNode:
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'composed_topic',
            self.subscriber_callback,
            10
        )

    def subscriber_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create نوڈز
    pub_node = PublisherNode()
    sub_node = SubscriberNode()
    
    # Create executor اور add نوڈز
    executor = SingleThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()
```
## babٹbr ی ط r یقہ ک ari

1
2
3
4.
5.

## خ LAA صہ

یہ الل l ی ں ا کے کے کے ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ذی ذی ذی ذی ذی ذی ذی ذی ذی ذی ذی ہ ہm juiu کے juiز mabaun bunahadaidadah mowaauladiad mowaaulad muchaulaad maunat maunat maus ، ، ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک نیچے ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک نیچے ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ہیں ک ہیں ک ک ک ک ک ک ک ک
