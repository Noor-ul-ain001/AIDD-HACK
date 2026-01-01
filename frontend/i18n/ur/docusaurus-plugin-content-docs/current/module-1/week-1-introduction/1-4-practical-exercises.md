---


sidebar_position: 4
difficulty: beginner


---

# 1.4: ROS 2 عملی مشقیں

## جائزہ

یہ ذیلی ماڈیول ROS 2 کے بنیادی تصورات کو عملی انداز میں سمجھنے کے لیے مشقیں فراہم کرتا ہے۔

## سیکھنے کے مقاصد

اس ذیلی ماڈیول کے اختتام تک، آپ کریں گے:
- ایک بنیادی ROS 2 نوڈ تشکیل دیں
- ROS 2 ٹاپکس کا استعمال کرکے مواصلات کریں
- ROS 2 سروسز کے ساتھ کام کریں
- ROS 2 launch files استعمال کریں

## مشق 1: بنیادی ROS 2 نوڈ

### مشق کا تعارف

آئیں ایک بنیادی ROS 2 نوڈ بنا کر شروع کریں جو ایک سادہ پیغام چھاپتا ہے۔

### اقدامات

1. ایک نیا ROS 2 پیکیج تشکیل دیں:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_pubsub --dependencies rclpy std_msgs
```

2. `py_pubsub/py_pubsub/publisher_member_function.py` میں درج ذیل کوڈ شامل کریں:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. پروجیکٹ کو بنانے کے لیے:
```bash
cd ~/ros2_ws
colcon build --packages-select py_pubsub
```

4. نوڈ کو چلانے کے لیے:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run py_pubsub publisher_member_function
```

## مشق 2: ROS 2 Subscriber

### مشق کا تعارف

Subscriber نوڈ بنائیں جو پبلشر کے پیغامات کو موصول کرے۔

### اقدامات

1. `py_pubsub/py_pubsub/subscriber_member_function.py` فائل بنا کر درج ذیل کوڈ شامل کریں:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. دوسرے ٹرمنل میں subscriber چلائیں:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run py_pubsub subscriber_member_function
```

## مشق 3: ROS 2 Service Client Server

### مشق کا تعارف

سروس کلائنٹ اور سرور کے درمیان مواصلات کا عمل دیکھیں۔

### اقدامات

1. ایک نیا پیکیج تشکیل دیں:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_srvclient --dependencies rclpy std_msgs example_interfaces
```

2. `py_srvclient/py_srvclient/service_member_function.py`:

```python
import sys
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main():
    rclpy.init()
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. `py_srvclient/py_srvclient/client_member_function.py`:

```python
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info('Result of add_two_ints: %d' % response.sum)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

4. سروس چلائیں:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run py_srvclient service_member_function
```

5. کلائنٹ چلائیں:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run py_srvclient client_member_function 5 3
```

## مشق 4: Launch File کا استعمال

### مشق کا تعارف

Launch file استعمال کریں تاکہ ایک ہی کمانڈ سے کئی نوڈز چلائے جا سکیں۔

### اقدامات

1. `py_pubsub/launch` ڈائریکٹری بنا دیں اور `talker_listener.launch.py` فائل بنائیں:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_pubsub',
            executable='publisher_member_function',
            name='talker',
            output='screen'
        ),
        Node(
            package='py_pubsub',
            executable='subscriber_member_function',
            name='listener',
            output='screen'
        )
    ])
```

2. launch file چلائیں:
```bash
cd ~/ros2_ws
colcon build --packages-select py_pubsub
source install/setup.bash
ros2 launch py_pubsub talker_listener.launch.py
```

## مشق 5: ROS 2 Tools کا استعمال

### مشق کا تعارف

ROS 2 کے مختلف ٹولز کی جانچ کریں۔

### اقدامات

1. نوڈس کی فہرست حاصل کریں:
```bash
ros2 node list
```

2. ٹاپکس کی فہرست حاصل کریں:
```bash
ros2 topic list
```

3. کسی ٹاپک کا امتحان کریں:
```bash
ros2 topic echo /topic
```

4. کسی ٹاپک کو ڈیٹا بھیجیں:
```bash
ros2 topic pub /topic std_msgs/msg/String "data: hello"
```

5. سرورسز کی فہرست حاصل کریں:
```bash
ros2 service list
```

## خلاصہ

یہ مشقیں ROS 2 کے بنیادی تصورات کے عملی استعمال کو ظاہر کرتی ہیں:
- نوڈس اور مواصلات
- ٹاپکس، سرورسز، اور ایکشنز
- Launch files
- ROS 2 ٹولز

آپ اب ROS 2 کے ساتھ مستقل طور پر کام کرنے کے لیے تیار ہیں۔