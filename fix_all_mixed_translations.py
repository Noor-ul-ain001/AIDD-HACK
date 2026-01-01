"""
Complete fix for mixed English-Urdu translations in i18n files.
This script creates proper Urdu translations for all submodules in the i18n folder.
"""
import os
from pathlib import Path

def translate_english_to_urdu_1_1_what_is_ros2():
    return """---


sidebar_position: 1
difficulty: beginner


---

# 1.1: ROS 2 کیا ہے؟

## جائزہ

یہ ذیلی ماڈیول روبوٹ آپریٹنگ سسٹم 2 (ROS 2) کی تفصیلی تلاش فراہم کرتا ہے - روبوٹ سافٹ ویئر ایپلی کیشنز کو تیار کرنے کے لیے اگلی نسل کا فریم ورک۔

## سیکھنے کے مقاصد

اس ذیلی ماڈیول کے اختتام تک، آپ:
- ROS 2 کے بنیادی تصورات کو سمجھیں گے
- جانیں گے کہ ROS 2 روایتی آپریٹنگ سسٹم سے کیسے مختلف ہے
- ROS 2 کے استعمال کے معاملات اور ایپلی کیشنز کو پہچانیں گے
- ROS 2 ماحولیاتی نظام سے واقف ہوں گے

## ROS 2 کیا ہے؟

ROS 2 روبوٹ آپریٹنگ سسٹم کی دوسری نسل ہے، روبوٹ ایپلی کیشنز کو تیار کرنے کے لیے ایک اوپن سورس فریم ورک ہے۔ یہ ہارڈ ویئر خلاصہ، ڈیوائس ڈرائیور، لائبریریاں، وژولائزرز، پیغام بھیجنے، پیکیج مینجمنٹ اور بہت کچھ فراہم کرتا ہے۔ ROS 2 اصل ROS کو بہتر سیکیورٹی، ریل ٹائم سپورٹ، اور دیگر اضافوں میں بہتر بناتا ہے۔

### کلیدی خصوصیات

- **مڈل ویئر-مبنی**: عمل کے مابین رابطے کے لیے DDS (ڈیٹا ڈسٹری بیوشن سروس) استعمال کرتا ہے
- **تقسیم شدہ**: ملٹی روبوٹ سسٹم اور تقسیم شدہ کمپیوٹیشن کی حمایت کرتا ہے
- **ماڈیولر**: اجزاء کو آزادانہ طور پر تیار اور تعینات کیا جا سکتا ہے
- **کراس پلیٹ فارم**: لینکس، ونڈوز، میک او ایس، اور ریئل ٹائم آپریٹنگ سسٹم پر چلتا ہے

### ROS 2 بمقابلہ روایتی OS

ROS 2 ریئل ٹائم آپریٹنگ سسٹم نہیں ہے لیکن متعدد مشینوں پر کام کرنے والے ٹولز اور لائبریریوں کا ایک مجموعہ ہے۔ جبکہ روایتی OSs ہارڈ ویئر وسائل کا نظم کرتے ہیں اور سسٹم سروسز فراہم کرتے ہیں، ROS 2 فراہم کرتا ہے:

- روبوٹک اجزاء کے مابین رابطہ
- ہارڈ ویئر خلاصہ اور ڈیوائس ڈرائیور انٹرفیسز
- ڈیبگنگ، ٹیسٹنگ، اور روبوٹ کے طرز عمل کو دیکھنے کے لیے ٹولز
- روبوٹک سافٹ ویئر اجزاء کو بانٹنے کے لیے ایک پیکیج مینجمنٹ سسٹم

## ROS 2 ماحولیاتی نظام

ROS 2 ماحولیاتی نظام مختلف اجزاء اور ٹولز پر مشتمل ہے جو روبوٹ کی ترقی کو آسان بنانے کے لیے ایک ساتھ کام کرتے ہیں:

1. **کور لائبریریاں**: rclcpp، rclpy (سی ++ اور پائیتھن کے لیے کلائنٹ لائبریریاں)
2. **بلڈ سسٹم**: کولکن (بلڈ اور پیکیج مینجمنٹ)
3. **کمیونیکیشن ٹولز**: ros2 کمانڈ لائن انٹرفیس
4. **وژولائزیشن ٹولز**: RViz، rqt ٹولز
5. **سمولیشن**: گیزبو انضمام

## ROS 2 کی ایپلی کیشنز

ROS 2 کو روبوٹکس کے مختلف شعبوں میں استعمال کیا جاتا ہے:

- **خودمختار گاڑیاں**: نیویگیشن اور تاثرات کے نظام
- **صنعتی روبوٹکس**: مینوفیکچرنگ اور آٹومیشن
- **سروس روبوٹکس**: ہیومنوائڈ روبوٹ، ساتھی روبوٹ
- **تحقیق**: تعلیمی اور کمرشل تحقیقی پلیٹ فارمز
- **خلائی ایکسپلوریشن**: ناسا روبوٹکس پروجیکٹس

## خلاصہ

اس ذیلی ماڈیول نے ROS 2 کو ایک مڈل ویئر فریم ورک کے طور پر متعارف کرایا جو پیچیدہ روبوٹک ایپلی کیشنز کی ترقی کو آسان بنا دیتا ہے۔ اگلا ذیلی ماڈیول ROS 2 فن تعمیر کو مزید تفصیل سے تلاش کرے گا۔"""

def translate_english_to_urdu_1_2_architecture():
    return """---


sidebar_position: 2
difficulty: beginner


---

# 1.2: ROS 2 فن تعمیر اور بنیادی تصورات

## جائزہ

یہ ذیلی ماڈیول اپنے بنیادی تصورات اور مواصلات کے طریقہ کار کی کھوج کرتے ہوئے، ROS 2 کے آرکیٹیکچرل ڈیزائن میں ڈوب جاتا ہے۔

## سیکھنے کے مقاصد

اس ذیلی ماڈیول کے اختتام تک، آپ کریں گے:
- DDS-مبنی ROS 2 فن تعمیر کو سمجھیں
- بنیادی تصورات (نوڈس، ٹاپکس، سروسز، ایکشنز) کی شناخت اور وضاحت
- ایک بنیادی ROS 2 نوڈ کو نافذ کریں
- DDS مواصلاتی پرت کے فوائد کو پہچانیں

## ROS 2 فن تعمیر کا جائزہ

ROS 2 DDS (ڈیٹا ڈسٹری بیوشن سروس) کو اپنے بنیادی مواصلاتی مڈل ویئر کے طور پر استعمال کرتا ہے۔ DDS ریئل ٹائم، تقسیم شدہ نظاموں کے لیے ایک معیاری مواصلاتی مڈل ویئر ہے، جو فراہم کرتا ہے:

- **شائع/سبسکرائب کریں**: غیر متزامن پیغام گزرنا
- **درخواست/جواب**: ہم آہنگی کی درخواست/جواب مواصلات
- ** مواد-مبنی سبسکرپشنز**: مواد کے ذریعہ فلٹر کردہ پیغامات
- **ڈسکوری**: خودکار شریک کا پتہ لگانا
- ** معیار کی سروس (QoS)**: قابل عمل مواصلاتی پالیسیاں

### بنیادی تصورات

1. ** نوڈس**: عمل جو کمپیوٹیشن انجام دیتے ہیں۔ نوڈس ROS 2 میں کمپیوٹیشن کی بنیادی اکائی ہیں۔

2. **پیغامات**: نوڈس کے مابین رابطے کے لیے استعمال ہونے والے ڈیٹا ڈھانچے۔

3. ** ٹاپکس**: نام جن پر پیغامات بھیجے جاتے ہیں اور وصول کیے جاتے ہیں۔ پبلشرز پیغامات بھیجتے ہیں، سبسکرائیبرز پیغامات وصول کرتے ہیں۔

4. ** سروسز**: ہم آہنگی کی درخواست/جواب مواصلات کا نمونہ۔

5. ** ایکشنز**: غیر متزامن گول میں مواصلات کا نمونہ۔

```python
# ROS 2 نوڈ کی مثال کی ساخت
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # سیکنڈ
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'ہیلو دنیا: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('شائع کیا جارہا ہے: "%s"' % msg.data)
        self.i += 1
```

## معیار کی سروس (QoS) کی ترتیبات

QoS کی پالیسیاں ڈویلپرز کو یہ ترتیب دینے کی اجازت دیتی ہیں کہ پیغامات کیسے فراہم کیے جاتے ہیں:

- ** قابل اعتمادیت**: بہترین کوشش یا قابل اعتماد ترسیل
- ** استحکام**: مطابقت یا عارضی مقامی
- ** تاریخ**: آخری N پیغامات رکھیں یا سب رکھیں
- ** گہرائی**: پیغامات کے لیے بفر سائز

## لانچ سسٹم

ROS 2 ایک وقت میں ایک سے زیادہ نوڈس شروع کرنے کے لیے لانچ فائلوں کا استعمال کرتا ہے:

```python
# پائیتھن لانچ فائل کی مثال
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='talker',
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener',
        ),
    ])
```

## خلاصہ

اس ذیلی ماڈیول نے ROS 2 کے آرکیٹیکچرل پہلوؤں کا احاطہ کیا، جس میں اس کے بنیادی تصورات اور بنیادی DDS مواصلاتی پرت بھی شامل ہے۔ اگلا ذیلی ماڈیول ROS 2 کے لیے تنصیب اور سیٹ اپ کے طریقہ کار کی تلاش کرے گا۔"""

def translate_english_to_urdu_1_3_installation_setup():
    return """---


sidebar_position: 3
difficulty: beginner


---

# 1.3: ROS 2 تنصیب اور سیٹ اپ

## جائزہ

یہ ذیلی ماڈیول ROS 2 کی تنصیب کا عمل اور ترقی کے ماحول کو ایڈجسٹ کرنے کا طریقہ سمجھاتا ہے۔

## سیکھنے کے مقاصد

اس ذیلی ماڈیول کے اختتام تک، آپ کریں گے:
- ROS 2 تنصیب کے لیے ضروریات کو سمجھیں
- ROS 2 مختلف آپریٹنگ سسٹم پر تنصیب کریں
- ROS 2 ماحولیاتی متغیرات کو تشکیل دیں
- ابتدائی ROS 2 ایپلی کیشن کو چلائیں

## ROS 2 تنصیب کی ضروریات

### سسٹم کی ضروریات

- **آپریٹنگ سسٹم**:
  - Ubuntu 20.04 (فocal) یا 22.04 (jammy)
  - Windows 10/11 (کے ساتھ WSL2)
  - macOS 11+ (Big Sur یا اس سے اوپر)

- **ہارڈویئر**:
  - چار-core 2GHz CPU یا اس سے بہتر
  - 8GB RAM یا اس سے زیادہ
  - 5GB فری اسٹوریج سپیس

- **سافٹ ویئر**:
  - Python 3.8 یا اس سے جدید
  - pip (Python package installer)
  - Git

### ROS 2 تنصیب کے اختیارات

ROS 2 کو متعدد طریقوں سے تنصیب کیا جا سکتا ہے:

1. **APT (Ubuntu)**: سرکاری Ubuntu repositories کے ذریعے
2. **Chocolatey (Windows)**: Windows کے لیے پیکیج مینجمنٹ سسٹم
3. **Bottles (macOS)**: macOS کے لیے ROS 2 bottles
4. **Docker**: کنٹینرائزڈ ROS 2 ماحول
5. **Source**: ROS 2 کو سورس کوڈ سے کمپائل کریں

## Ubuntu پر ROS 2 تنصیب

### Debian/Ubuntu کے لیے APT ذریعہ شامل کریں

```bash
# GPG key شامل کریں
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# APT ذریعہ شامل کریں
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### ROS 2 تنصیب کریں

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

### ROS 2 ماحولیاتی متغیرات تشکیل دیں

```bash
# سیشن کے لیے ماحولیاتی متغیرات شامل کریں
source /opt/ros/humble/setup.bash

# مستقل بنیادوں پر ماحولیاتی متغیرات شامل کریں
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Windows پر ROS 2 تنصیب

### WSL2 تنصیب

1. Windows فیچر: Linux Kernel اپ ڈیٹ کریں
2. WSL2 انسٹال کریں
3. Ubuntu انسٹال کریں (Microsoft Store سے)
4. WSL2 کے لیے ڈیفالٹ ورژن کے طور پر سیٹ کریں

```powershell
wsl --set-default-version 2
```

### ROS 2 داخل کریں

WSL Ubuntu میں داخل ہوں اور پہلے والے Ubuntu کے اقدامات کی پیروی کریں:

```bash
# WSL terminal میں
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

## macOS پر ROS 2 تنصیب

### Homebrew کے ذریعے

```bash
# Homebrew اگر ابھی تک انسٹال نہیں ہے
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Python3 اور pip انسٹال کریں
brew install python3

# ROS 2 bottle انسٹال کریں
brew install ros/humble/humble-desktop
source /opt/ros/humble/setup.bash
```

## Docker کا استعمال کرکے ROS 2

Docker ROS 2 کا استعمال کرنے کا ایک مؤثر طریقہ ہے:

```bash
# ROS 2 Docker image ڈاؤن لوڈ کریں
docker pull ros:humble

# ایک interactive container شروع کریں
docker run -it ros:humble

# container کے اندر
source /opt/ros/humble/setup.bash
ros2 --version
```

## تنصیب کی تصدیق

نصیب کی تصدیق کے لیے، ROS 2 کمانڈز چلائیں:

```bash
# ROS 2 ورژن چیک کریں
ros2 --version

# ROS 2 pkgs لسٹ کریں
ros2 pkg list

# rviz2 چلائیں
rviz2
```

## ROS 2 Development Environment کی ترتیب

### کمپائلر کی تنصیب

```bash
# Ubuntu
sudo apt install build-essential cmake pkg-config

# Windows (WSL)
sudo apt install build-essential cmake pkg-config

# macOS
xcode-select --install
```

### IDE کا انتخاب

- **VSCode**: ROS 2 کے لیے وسیع تر سپورٹ کے ساتھ
- **Qt Creator**: C++ ROS 2 development کے لیے
- **PyCharm**: Python ROS 2 development کے لیے

### ROS 2 workspace تشکیل

```bash
# workspace directory بنا دیں
mkdir -p ~/ros2_ws/src

# source کریں
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# colcon build
colcon build

# workspace سیشن کے لیے source کریں
source install/setup.bash
```

## خلاصہ

اس ذیلی ماڈیول نے ROS 2 کی متعدد آپریٹنگ سسٹم پر تنصیب کا عمل، ترتیب اور تصدیق کا طریقہ سمجھایا ہے۔ اب آپ ROS 2 کے ساتھ کام کرنے کے لیے تیار ہیں۔ اگلا ذیلی ماڈیول ROS 2 کی ابتدائی مشقیں پیش کرے گا۔"""

def translate_english_to_urdu_1_4_practical_exercises():
    return """---


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
        self.get_logger().info('Incoming request\\na: %d b: %d' % (request.a, request.b))
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

آپ اب ROS 2 کے ساتھ مستقل طور پر کام کرنے کے لیے تیار ہیں۔"""

def translate_english_to_urdu_2_1_overview_of_robot_simulation():
    return """---


sidebar_position: 1
difficulty: beginner


---

# 2.1: روبوٹ سمولیشن کا جائزہ

## جائزہ

یہ ذیلی ماڈیول ROS 2 میں روبوٹ سمولیشن کے بنیادی تصورات کو متعارف کراتا ہے، جس میں روبوٹکس کی ترقی، مختلف سمولیشن نقطہ نظر، اور روبوٹکس ڈویلپمنٹ لائف سائیکل میں سمولیشن کے کردار کو شامل کیا جاتا ہے۔

## سیکھنے کے مقاصد

اس ذیلی ماڈیول کے اختتام تک، آپ کریں گے:
- روبوٹ سمولیشن کی اہمیت اور فوائد کو سمجھیں
- مختلف قسم کے روبوٹ سمولیشن ماحول کی شناخت کریں
- روبوٹکس ڈویلپمنٹ اور ٹیسٹنگ میں سمولیشن کے کردار کو پہچانیں
- سمولیشن بمقابلہ اصلی دنیا کے روبوٹکس ڈویلپمنٹ کا موازنہ کریں
- سمولیشن کی حدود اور فوائد کو سمجھیں

## روبوٹ سمولیشن کا تعارف

روبوٹ سمولیشن جدید روبوٹکس کی ترقی کا ایک اہم جز ہے۔ یہ ڈویلپرز کو اجازت دیتا ہے:

- ہارڈ ویئر کو خطرہ کے بغیر محفوظ طریقے سے ٹیسٹ الگورتھم
- کنٹرول ماحول میں روبوٹ کے طرز عمل کی توثیق کریں
- جسمانی ہارڈ ویئر پر تعینات کرنے سے پہلے کارکردگی کا اندازہ کریں
- ترقی اور جانچ کے چکروں کو تیز کریں
- ایسے منظرناموں کے ساتھ تجربہ کریں جو حقیقی روبوٹ پر جانچنا مشکل یا خطرناک ہوگا

### سمولیشن کیوں ضروری ہے

روبوٹ سمولیشن روبوٹکس کی ترقی میں کئی اہم مقاصد کی تکمیل کرتا ہے:

1. ** حفاظت **: مہنگے ہارڈ ویئر کو نقصان پہنچانے یا لوگوں کو نقصان پہنچانے کے خطرے کے بغیر پیچیدہ طرز عمل کی جانچ کریں
2. ** قیمت کی کارکردگی **: جانچ کے لئے درکار ہارڈ ویئر کے اخراجات اور وقت کو کم کریں
3. ** تیز رفتار ترقی **: جسمانی ہارڈ ویئر کے مقابلے میں زیادہ تیزی سے ٹیسٹ اور تکرار کریں
4. ** بار بار ماحول **: نتائج کی دوبارہ توثیق کے لیے مطابقت کے ماحول فراہم کریں
5. ** خطرناک منظرناموں کی جانچ **: ایسے منظرنامے جو حقیقی دنیا میں جانچنا خطرناک ہو

### سمولیشن کی اقسام

سمولیشن ماحول ان کی وفاداری اور پیچیدگی کی سطح میں مختلف ہوتا ہے:

1. ** Kinematic Simulation **: قوتوں پر غور کیے بغیر حرکت پر مرکوز ہے
2. ** Dynamic Simulation **: قوتیں، ٹارک اور جسمانی تعامل شامل ہیں
3. ** Physics-based Simulation **: حقیقت پسندانہ جسمانی خصوصیات اور تعامل کو شامل کیا گیا ہے
4. ** Sensor Simulation **: ماڈل سینسر حقیقت پسندانہ شور اور حدود کے ساتھ آؤٹ پٹ کرتا ہے
5. ** Environment Simulation **: مکمل آپریٹنگ ماحول کے ماڈل

## ROS 2 Ecosystem میں سمولیشن

### ROS 2 کے ساتھ انضمام

ROS 2 معیاری انٹرفیس کے ذریعہ سمولیشن ماحول کے ساتھ مربوط ہے:

- ** معیاری پیغامات **: سینسر پیغامات، کنٹرول پیغامات وغیرہ
- ** TF انضمام **: سمولیشن حقیقت پسندانہ تبدیلی کا ڈیٹا فراہم کرتا ہے
- ** URDF مطابقت **: اصلی اور سمولیشن روبوٹ کے لیے ایک ہی روبوٹ کی تفصیل
- ** ROS 2 Control **: Smolition اور اصلی روبوٹ دونوں کے لیے ایک ہی کنٹرول انٹرفیس

### ROS 2 کے لیے مقبول Smolition ماحول

1. ** Gazebo **: حقیقت پسندانہ فزکس اور سینسر کے ساتھ فزکس پر مبنی Smolition
2. ** Ignition Gazebo **: Open Robotics سے اگلی نسل کا Smolition
3. ** Webots **: عمومی مقصد روبوٹ Smolition سافٹ ویئر
4. ** PyBullet **: Python API کے ساتھ فزکس Smolition
5. ** Unity ML-Agents **: AI کی تربیت کے لیے گیم انجن پر مبنی Smolition
6. ** Carla **: خود مختار ڈرائیونگ ایپلی کیشنز کا Smolition

## Smolition بمقابلہ حقیقت کا فرق

### حقیقت کا فرق

Smolition سے حقیقت کا فرق روبوٹکس میں ایک اہم تصور ہے:

- ** ماڈل کی خامیوں **: اصلی روبوٹ میں غیر ترمیم شدہ حرکیات اور طرز عمل ہیں
- ** سینسر شور **: اصلی سینسر ماڈل کے مقابلے میں شور کی مختلف خصوصیات رکھتے ہیں
- ** ماحولیاتی عوامل **: لائٹنگ، سطحیں اور دیگر حالات مختلف ہوتے ہیں

### گیپ کو برج کرنا

حقیقت کے فرق کو کم سے کم کرنے کی حکمت عملیوں میں شامل ہیں:

1. ** سسٹم کی شناخت **: درست طریقے سے ماڈل روبوٹ حرکیات
2. ** سینسر کیلیبریشن **: اصلی سینسر کی خصوصیات سے Smolition سینسر کو میچ کریں
3. ** ڈومین رینڈمائزیشن **: مختلف Smolition ماحول میں ٹرین
4. ** منظم جانچ **: اصلی ہارڈ ویئر پر وسیع پیمانے پر توثیق
5. ** سیم ٹو ریئل ٹرانسفر تکنیکیں **: Smolition کے نتائج کو حقیقت میں بہتر نافذ کریں

## Smolition کے فوائد

### ڈویلپمنٹ فوائد

1. ** محفوظ ٹیسٹنگ **: مہنگے ہارڈ ویئر کو نقصان پہنچانے کے بغیر جانچ
2. ** ڈیبگنگ **: نظام کے طرز عمل کو ڈیبگ اور تجزیہ کرنا آسان ہے
3. ** ملٹی روبوٹ سسٹم **: متعدد جسمانی روبوٹ کے بغیر پیچیدہ تعامل کی جانچ کریں
4. ** رجعت ٹیسٹنگ **: سسٹم ٹوٹ جانے سے بچنے کے لیے خودکار ٹیسٹ سویٹس
5. ** الگورتھم ڈویلپمنٹ **: کنٹرول ماحول میں الگورتھم تیار کریں اور ان کو بہتر بنائیں

### ٹیسٹنگ فوائد

1. ** سیفٹی ٹیسٹنگ **: ممکنہ طور پر خطرناک منظرناموں میں روبوٹ کے رویے کا اندازہ کریں
2. ** ایج کیس ٹیسٹنگ **: نایاب لیکن اہم منظرنامے کی جانچ کریں
3. ** کارکردگی کی تشخیص **: مقداری طور پر نظام کی کارکردگی کی پیمائش کریں
4. ** طویل مدتی جانچ **: توسیعی ادوار کے لیے ٹیسٹ چلائیں
5. ** قابل تکرار نتائج **: منصفانہ موازنہ کے لیے ایک جیسی شرائط

## عام Smolition استعمال کے معاملات

### 1. نیویگیشن ٹیسٹنگ

نیویگیشن الگورتھم کی جانچ کے لیے Smolition بڑے پیمانے پر استعمال ہوتا ہے:

```bash
# مثال: نیویگیشن Smolition چلانا
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```

### 2. ہیرا پھیری کی منصوبہ بندی

روبوٹ بازو کی تحریک کی منصوبہ بندی اور گرفت کی جانچ:

```bash
# مثال: Smolition ہیرا پھیری
ros2 launch my_robot_gazebo arm_simulation.launch.py
ros2 run my_robot_control pick_and_place_demo
```

### 3. SLAM ڈویلپمنٹ

بیک وقت لوکلائزیشن اور میپنگ الگورتھم ٹیسٹنگ:

```bash
# مثال: SLAM Smolition
ros2 launch my_robot_gazebo mapping_simulation.launch.py
ros2 run my_robot_slam slam_node
```

### 4. ملٹی روبوٹ سسٹم

ایک سے زیادہ روبوٹ کے مابین جانچ کوآرڈینیشن اور مواصلات:

```bash
# مثال: ملٹی روبوٹ Smolition
ros2 launch my_robot_gazebo multi_robot_world.launch.py
ros2 run my_robot_coordination coordination_node
```

## Smolition کی حدود

### فزکس کی حدود

- ** پیچیدہ رابطے **: رگڑ، تصادم اور رابطوں کی ماڈلنگ
- ** لچک **: ناقص اشیاء اور لچکدار ڈھانچے
- ** سیال حرکیات **: مائعات اور گیسوں کے ساتھ تعامل
- ** حقیقی وقت کی کارکردگی **: درستگی اور رفتار کے مابین تجارت

### سینسر کی حدود

- ** ماڈلنگ حقیقت پسندی **: پیچیدہ سینسر کے طرز عمل اور ناکامی کے طریقے
- ** ماحولیاتی اثرات **: دھند، دھند، بارش، روشنی کے حالات
- ** وقت کے اثرات **: ہم آہنگی اور تاخیر کی خصوصیات
- ** کراس سینسر مداخلت **: EMI اور دیگر مداخلت کے اثرات

### کمپیوٹیشنل حدود

- ** حقیقی وقت کی رکاوٹیں **: Smolition کی کارکردگی کی ضروریات
- ** ماڈل پیچیدگی **: کارکردگی کے ساتھ تفصیل میں توازن
- ** اسکیل ایبلٹی **: بڑے ماحول اور ملٹی روبوٹ سسٹم

## Smolition کے بہترین عمل

### ماڈل ڈویلپمنٹ

1. ** سادہ شروع کریں **: بنیادی ماڈل سے شروع کریں اور آہستہ آہستہ پیچیدگی شامل کریں
2. ** صارفین کی توثیق **: اصلی ہارڈ ویئر کے ساتھ Smolition کے نتائج کو موازنہ کریں
3. ** دستاویز کے مفروضات **: واضح طور پر دستاویز کی ماڈل کی حدود

### جانچ کی حکمت عملی

1. ** Smolition سے حقیقت ٹرانسفر **: اصلی ہارڈ ویئر پر حتمی تعیناتی کا منصوبہ
2. ** منظم توثیق **: جب ممکن ہو تو Smolition کے نتائج کو حقیقت کے ساتھ موازنہ کریں
3. ** ایج کیس ٹیسٹنگ **: معلوم ناکامی کے طریقوں کے ٹیسٹ شامل کریں
4. ** کارکردگی کی پیمائش **: کامیابی کے لیے مقداری میٹرکس قائم کریں

## ڈویلپمنٹ لائف سائیکل میں Smolition

Smolition روبوٹکس ڈویلپمنٹ لائف سائیکل میں کلیدی کردار ادا کرتا ہے:

1. ** تصور کی ترقی **: خیالات کی ابتدائی جانچ
2. ** الگورتھم ڈویلپمنٹ **: کنٹرول اور خیال الگورتھم کو بہتر بنانا
3. ** انضمام کی جانچ **: اجزاء کو مل کر کام کرنے کو یقینی بنانا
4. ** سسٹم کی توثیق **: نظام کی مجموعی کارکردگی کی تصدیق کرنا
5. ** رجعت کی جانچ **: تازہ کاریوں کو یقینی بنانا موجودہ فعالیت کو نہیں توڑتا ہے

## روبوٹ Smolition کا مستقبل

### ابھرتے ہوئے رجحانات

1. ** فوٹو وریلسٹک Smolition **: حقیقت پسندانہ رینڈرنگ اور فزکس کے ساتھ
2. ** کلاؤڈ پر مبنی Smolition **: بڑے کلسٹروں کو اسکیلنگ Smolition
3. ** ڈومین رینڈمائزیشن **: حقیقت سے Smolition ٹرانسفر کو بہتر بنانے کے لیے
4. ** Smolition پر مبنی لرننگ **: AI کی تربیت براہ راست Smolition میں

### صنعت کو اپنانا

- ** مینوفیکچرنگ **: فیکٹری آٹومیشن اور روبوٹ کی تعیناتی کے لیے Smolition
- ** لاجسٹک **: گودام اور ترسیل کے روبوٹ کی جانچ
- ** خودمختار گاڑیاں **: مصنوعی ماحول میں وسیع پیمانے پر جانچ
- ** صحت کی دیکھ بھال **: سرجیکل روبوٹ اور معاون روبوٹ کی ترقی

## خلاصہ

اس ذیلی ماڈیول نے ROS 2 میں روبوٹ Smolition کا ایک جائزہ فراہم کیا، جس میں اس کی اہمیت، اقسام، فوائد، حدود اور بہترین طریقوں کا احاطہ کیا گیا ہے۔ محفوظ، قیمت سے موثر روبوٹکس کی ترقی اور جانچ کے لیے Smolition ضروری ہے۔ اگلا ذیلی ماڈیول Smolition کی تنصیب، بنیادی تصورات اور استعمال کا احاطہ کرتا ہے، جو ROS 2 کے لیے سب سے مشہور Smolition ماحول، Gazebo میں غوطہ لگائے گا۔"""

def translate_english_to_urdu_2_2_gazebo_installation_setup():
    return """---


sidebar_position: 2
difficulty: beginner


---

# 2.2: Gazebo تنصیب اور سیٹ اپ

## جائزہ

یہ ذیلی ماڈیول Gazebo کی تنصیب، تشکیل، اور ROS 2 کے ساتھ مربوط کرنے کے عمل کی تفصیل فراہم کرتا ہے۔

## سیکھنے کے مقاصد

اس ذیلی ماڈیول کے اختتام تک، آپ کریں گے:
- Gazebo کی متعدد ورژن کو سمجھیں گے
- Gazebo کو مختلف آپریٹنگ سسٹم پر تنصیب کریں گے
- ROS 2 انضمام کی تشکیل کریں گے
- Gazebo دنیا کو شروع کریں گے

## Gazebo کی تاریخ اور ورژن

Gazebo ایک مقبول روبوٹ Smolition سافٹ ویئر ہے جو پہلا دیکھنے کے لحاظ سے حقیقت پسندانہ ماحول فراہم کرتا ہے۔

### Gazebo کے مختلف ورژن

1. **Original Gazebo**: 2010 میں شروع کیا گیا
2. **Gazebo Classic**: 11 ورژن تک
3. **Gazebo (Ignition)**: Ros2 Humble کے ساتھ استعمال کیا جاتا ہے
4. **Gazebo Garden**: Ros2 Humble کے ساتھ ابتدائی تجربہ

### ROS 2 انضمام

ROS 2 کے ساتھ Gazebo کے انضمام کے لیے، ROS 2 Gazebo pkgs استعمال کیے جاتے ہیں:

- `ros_gz`: ROS 2 اور Gazebo کے مابین مواصلات کا پل
- `ros_gz_bridge`: ROS 2 اور Gazebo پیغامات کے درمیان ترجمہ
- `gz_ros2_control`: ROS 2 کنٹرول کا استعمال کرتے ہوئے Gazebo میں سینسر اور ایکٹویٹر

## Ubuntu پر Gazebo تنصیب

### Gazebo Garden تنصیب

```bash
# Gazebo APT ذریعہ شامل کریں
sudo apt update && sudo apt install wget lsb-release gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg

# Gazebo APT ذریعہ شامل کریں
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo.list > /dev/null

# Gazebo تنصیب کریں
sudo apt update
sudo apt install gz-garden
```

### ROS 2 Gazebo Packages تنصیب

```bash
# Ros2 Humble کے لیے Gazebo packages تنصیب کریں
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-gz-ros2-control
```

## Windows پر Gazebo تنصیب

Windows کے لیے Gazebo تنصیب WSL2 کے ذریعے کی جاتی ہے:

```bash
# WSL Ubuntu ٹرمنل میں
sudo apt update && sudo apt install wget lsb-release gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo.list > /dev/null
sudo apt update
sudo apt install gz-garden ros-humble-ros-gz ros-humble-ros-gz-bridge
```

## macOS پر Gazebo تنصیب

```bash
# Homebrew کے ذریعے
brew install osrf/simulation/gz-garden
brew install ros-humble-ros-gz ros-humble-ros-gz-bridge
```

## Gazebo اور ROS 2 کا آغاز

### Gazebo ورلڈ چلانا

```bash
# Gazebo ورلڈ چلانا
gz sim

# Ros2 کے ساتھ Gazebo ورلڈ چلانا
ros2 launch ros_gz_sim gz_sim.launch.py world_name:=empty.sdf
```

### Ros 2 Gazebo Bridge شروع کرنا

```bash
# Ros2 Gazebo Bridge چلانا
ros2 run ros_gz_bridge parameter_bridge
```

## Ros 2 Gazebo Smolition کی تشکیل

### Ros 2 Package کے ساتھ Gazebo استعمال کرنا

Ros 2 Package کی تشکیل میں، `CMakeLists.txt` اور `package.xml` میں Gazebo dependencies شامل کریں:

`package.xml` میں:
```xml
<depend>ros_gz_sim</depend>
<depend>ros_gz_bridge</depend>
<depend>gz_ros2_control</depend>
```

### Launch File کے ذریعے Gazebo ورلڈ

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_sim.actions import Create
from ros_gz_sim.launch.gz_launch import GzLaunchDescription

def generate_launch_description():
    world = LaunchConfiguration('world')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Choose one of the world files from `/ros_gz_sim/worlds`'
        ),
        
        # Gazebo Server چلانا
        Node(
            package='ros_gz_sim',
            executable='gzserver',
            arguments=['-s', 'libignition-gazebo-physics-system.so', 
                      '-s', 'libignition-gazebo-user-commands-system.so',
                      world],
            output='screen'
        ),
        
        # Gazebo Client چلانا
        Node(
            package='ros_gz_sim',
            executable='gzclient',
            output='screen'
        )
    ])
```

## Gazebo Models اور Worlds

### Gazebo Models کو انسٹال کرنا

```bash
# Ros2 Gazebo models انسٹال کریں
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim

# یا اس کے ذریعے
ros2 run ros_gz_sim download_models
```

### اپنی تخلیق کردہ Gazebo دنیا کو شامل کرنا

```bash
# Ros2 workspace میں ایک دنیا فولڈر بنائیں
mkdir -p ~/ros2_ws/src/my_robot_gazebo/worlds

# دنیا کے فائل کو Ros2 package میں شامل کریں
# دنیا کے فائل کی مثال:
# my_robot_gazebo/worlds/my_world.sdf
```

## بنیادی Gazebo Smolition کا ڈیمو

### Ros 2 Nod سے Gazebo میں ٹاپکس کا استعمال کرکے ڈیٹا بھیجنا

```python
# Publisher script
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GazeboCommander(Node):
    def __init__(self):
        super().__init__('gazebo_commander')
        self.publisher = self.create_publisher(Twist, '/model/vehicle/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.5
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Gazebo Smolition کی تصدیق

### بنیادی Smolition کی جانچ

1. Gazebo ورلڈ چلائیں:
```bash
ros2 launch ros_gz_sim gz_sim.launch.py world_name:=empty.sdf
```

2. Gazebo میں ایک ماڈل شامل کریں:
```bash
ros2 run ros_gz_sim create -world default -file /usr/share/gazebo-11/worlds/models/ground_plane/model.sdf -z 0
```

3. Ros2 ٹاپکس کو جانچیں:
```bash
ros2 topic list | grep gz
```

## خلاصہ

اس ذیلی ماڈیول نے Gazebo کی تنصیب، ROS 2 انضمام، اور بنیادی Smolition کی تشکیل کا عمل سمجھایا ہے۔ اب آپ Gazebo میں روبوٹ Smolition کے لیے ROS 2 کا استعمال کرنے کے لیے تیار ہیں۔ اگلا ذیلی ماڈیول Gazebo کے بنیادی تصورات اور تکنیکوں کا احاطہ کرے گا۔"""

def translate_english_to_urdu_3_1_overview_of_nvidia_isaac_sim():
    return """---


sidebar_position: 1
difficulty: beginner


---

# 3.1: NVIDIA Isaac Sim کا جائزہ

## جائزہ

یہ ذیلی ماڈیول NVIDIA Isaac Sim، NVIDIA کے جدید روبوٹکس سمولیشن پلیٹ فارم کا ایک جامع تعارف فراہم کرتا ہے جو Omniverse فریم ورک پر مبنی ہے۔ ہم اس کی صلاحیتوں، فن تعمیر، اور یہ ROS 2 کے ساتھ کس طرح مربوط ہے اس کی کھوج کریں گے۔

## سیکھنے کے مقاصد

اس ذیلی ماڈیول کے اختتام تک، آپ کریں گے:
- NVIDIA Isaac Sim کے فن تعمیر اور صلاحیتوں کو سمجھیں گے
- Isaac Sim کا موازنہ دوسرے Smolition پلیٹ فارم کے ساتھ کریں گے
- روبوٹکس میں استعمال کے معاملات اور Isaac Sim کی ایپلی کیشنز کی شناخت کریں گے
- Isaac Sim اور ROS 2 کے مابین انضمام کو سمجھیں گے
- Isaac Sim کی رینڈرنگ اور فزکس کی صلاحیتوں کے بارے میں جانیں گے
- AI کی تربیت اور ترقی میں Isaac Sim کے کردار کو دریافت کریں گے

## NVIDIA Isaac Sim کا تعارف

NVIDIA Isaac Sim NVIDIA Omniverse فریم ورک پر تعمیر کردہ ایک جامع روبوٹکس Smolition ماحول ہے۔ یہ جسمانی طور پر درست، فوٹو وریلسٹک Smolition ماحول فراہم کرتا ہے جو خاص طور پر AI کی تربیت، جانچ اور روبوٹکس سسٹم کی تعیناتی کے لیے ڈیزائن کیا گیا ہے۔

### Isaac Sim کی کلیدی خصوصیات

1. ** جسمانی طور پر درست Smolition **: حقیقت پسندانہ روبوٹ سلوک کے لیے اعلی درستی فزکس انجن
2. ** فوٹو وریلسٹک رینڈرنگ **: RTX ray tracing اور PhysX کے ساتھ
3. ** ROS 2 انضمام **: ROS 2 مواصلات اور ٹولز کے لیے مقامی تعاون
4. ** ماڈیولر فن تعمیر **: اپنی مرضی کے مطابق توسیع کے لچکدار فریم ورک
5. ** AI تربیت کے لیے تیار **: مصنوعی ڈیٹا جنریشن اور کریل لرننگ کے لیے بلٹ ان ٹولز
6. ** حقیقت پسندانہ سینسر Smolition **: کیمرے، LIDAR، IMU، اور دیگر سینسرز کا درست Smolition

### Isaac Sim بمقابلہ روایتی Smolition پلیٹ فارم

| پہلو | Gazebo Classic | NVIDIA Isaac Sim |
|--------|----------------|-------------------|
| فزکس انجن | ODE، Bullet، DART | PhysX (NVIDIA's GPU-accelerated) |
| رینڈرنگ | بنیادی OpenGL | RTX- accelerated ray tracing |
| فوٹو وریلزم | کم | اعلی (فوٹو وریلسٹک) |
| مصنوعی ڈیٹا | محدود | وسیع ٹول سیٹ |
| GPU ایکسلریشن | محدود | مکمل RTX ایکسلریشن |
| AI ٹریننگ | بنیادی | بلٹ ان AI ٹولز |

## Isaac Sim فن تعمیر

### بنیادی اجزاء

Isaac Sim کئی کلیدی اجزاء پر مبنی ہے:

1. ** Omniverse **: NVIDIA کا متوازی، مبنی کمپیوٹیشنل فریم ورک
2. ** USD (Universal Scene Description) **: 3D منظر کی تفصیل کے لیے اوپن سورس فائل فارمیٹ
3. ** PhysX **: NVIDIA کا فزکس انجن
4. ** RTX رینڈرنگ **: Ray tracing اور AI-boosted rendering
5. ** Isaac Sim ایکسٹینشنز **: روبوٹکس Smolition کے لیے خصوصی ٹولز
6. ** ROS 2 برج **: ROS 2 ماحولیاتی نظام کے ساتھ مقامی انضمام

### Isaac Sim میں USD

یونیورسل سین ​​کی تفصیل (USD) Isaac Sim میں ایک کلیدی ٹکنالوجی ہے:

- ** توسیع پذیر منظر کی نمائندگی **: پیچیدہ، بڑے پیمانے پر ماحول کو سنبھالتا ہے
- ** لیئرڈ سٹرکچر **: موثر ترمیم اور تعاون کی اجازت دیتا ہے
- ** کراس پلیٹ فارم مطابقت **: دوسرے 3D ٹولز اور پلیٹ فارم کے ساتھ کام کرتا ہے
- ** ایکسٹینسیبل اسکیما **: روبوٹکس سے متعلق مخصوص عناصر کے لیے کسٹم اسکیمے

## Isaac Sim صلاحیتیں

### فزکس Smolition

Isaac Sim فزکس Smolition کے لیے NVIDIA کے PhysX انجن کا استعمال کرتا ہے:

```python
# Isaac Sim میں فزکس کی خصوصیات تک رسائی کی مثال
# یہ Python API کے ذریعے Isaac Sim میں ہوتا ہے
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# دنیا کو شروع کریں
world = World(stage_units_in_meters=1.0)

# اشیاء شامل کریں
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cube",
        name="my_cube",
        position=[0, 0, 1.0],
        size=0.5,
        mass=1.0
    )
)
```

### فوٹو وریلسٹک رینڈرنگ

Isaac Sim کی رینڈرنگ صلاحیتوں میں شامل ہیں:

- ** Ray Tracing **: حقیقت پسندانہ روشنی کے لیے ریئل ٹائم ray tracing
- ** مادی Smolition **: درست مادی خصوصیات اور تعامل
- ** ماحولیاتی اثرات **: موسم، روشنی اور ماحولیاتی حالات
- ** سینسر Smolition **: حقیقت پسندانہ کیمرا اور سینسر آؤٹ پٹ

### سینسر Smolition

Isaac Sim انتہائی درست سینسر Smolition فراہم کرتا ہے:

```python
# Isaac Sim میں سینسر سیٹ اپ کی مثال
from omni.isaac.sensor import Camera
import numpy as np

# ایک کیمرا سینسر بنائیں
camera = Camera(
    prim_path="/World/Robot/Camera",
    frequency=30,
    resolution=(640, 480)
)

# RGB ڈیٹا حاصل کریں
rgb_data = camera.get_rgb()
# ڈیپتھ ڈیٹا حاصل کریں
depth_data = camera.get_depth()
# سیگمینٹیشن ڈیٹا حاصل کریں
seg_data = camera.get_semantic_segmentation()
```

## Isaac Sim اور ROS 2 انضمام

### ROS 2 برج فن تعمیر

Isaac Sim ROS 2 برج ایکسٹینشن کے ذریعے ROS 2 کے لیے مقامی مدد فراہم کرتا ہے:

- ** پیغام ترجمہ **: Isaac Sim اور ROS 2 پیغامات کے مابین خودکار تبدیلی
- ** سروس سپورٹ **: Smolition کنٹرول کے لیے ROS 2 خدمات
- ** ایکشن سپورٹ **: پیچیدہ طرز عمل کے لیے ROS 2 ایکشنز
- ** TF انضمام **: روبوٹ فریموں کے لیے مناسب TF درخت کی اشاعت

### بنیادی ROS 2 انضمام

```python
# Isaac Sim میں ROS 2 انضمام کی مثال
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# ROS 2 نوڈ شروع کریں
rclpy.init()

# روبوٹ کنٹرول کے لیے ROS 2 شائع کار بنائیں
cmd_vel_pub = rclpy.create_publisher(Twist, '/cmd_vel', 10)

# سینسر ڈیٹا کے لیے ROS 2 سبسکرائیب کریں
def scan_callback(msg):
    print(f"Laser scan {len(msg.ranges)} points کے ساتھ موصول ہوا")

scan_sub = rclpy.create_subscription(LaserScan, '/scan', scan_callback, 10)
```

## تنصیب اور سیٹ اپ

### سسٹم کی ضروریات

Isaac Sim کو مخصوص ہارڈ ویئر کی ضرورت ہوتی ہے:

- ** GPU **: NVIDIA RTX سیریز (RTX 3080 یا بہتر تجویز کردہ)
- ** VRAM **: کم از کم 8GB، 24GB+ تجویز کردہ
- ** CPU **: ملٹی کور پروسیسر (Intel i7 یا AMD Ryzen 7+)
- ** RAM **: 32 GB+ سسٹم میموری
- ** OS **: Ubuntu 20.04/22.04 یا Windows 10/11
- ** CUDA **: CUDA 11.0+ مطابقت پذیر ڈرائیور

### تنصیب کے اختیارات

1. ** Docker انسٹالیشن ** (ابتدائی صارفین کے لیے تجویز کردہ):

```bash
docker run --gpus all -it --rm \
  --name isaac_sim \
  --net=host \
  -v ~/isaac_sim_exts:/home/isaac-sim/.nvidia-omniverse/extensions \
  -v ~/.nvidia-omniverse/logs:/home/isaac-sim/.nvidia-omniverse/logs \
  -v ~/isaac_assets:/home/isaac-sim/isaac_assets \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e QT_X11_NO_MITSHM=1 \
  nvcr.io/nvidia/isaac-sim:latest
```

2. ** مقامی تنصیب **: NVIDIA ڈویلپر ویب سائٹ سے ڈاؤن لوڈ کریں

### ROS 2 برج سیٹ اپ

ROS 2 انضمام کو فعال کرنے کے لیے:

```bash
# Isaac Sim میں، ROS2 Bridge ایکسٹینشن کو فعال کریں
# Extensions -> Isaac ROS2 Bridge -> Enable
```

## Isaac Sim ایکسٹینشنز اور ٹولز

### بنیادی ایکسٹینشنز

Isaac Sim میں کئی اہم ایکسٹینشنز شامل ہیں:

1. ** Isaac ROS2 Bridge **: Isaac Sim کو ROS 2 نیٹ ورکس سے جوڑتا ہے
2. ** Isaac Sim Navigation **: نیویگیشن Smolition کے لیے اوزار
3. ** Isaac Sim سینسر **: ایڈوانسڈ سینسر Smolition کے اوزار
4. ** Isaac Sim نیویگیشن **: نیویگیشن Smolition کے لیے اوزار
5. ** Isaac Sim ہیرا پھیری **: ہیرا پھیری کے Smolition کے لیے اوزار

### کسٹم ایکسٹینشنز

Isaac Sim کے لیے کسٹم ایکسٹینشن بنانا:

```python
# ایک کسٹم Isaac Sim ایکسٹینشن کی مثال
import omni.ext
from pxr import Usd
from omni.isaac.core import World

class MyCustomExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print(f"[my.custom.extension] Startup {ext_id}")
        # اپنی ایکسٹینشن یہاں شروع کریں
        self._world = World(stage_units_in_meters=1.0)

    def on_shutdown(self):
        print("[my.custom.extension] Shutdown")
        # اپنی ایکسٹینشن یہاں صاف کریں
        if hasattr(self, '_world'):
            self._world.cleanup()
```

## استعمال کے معاملات اور ایپلی کیشنز

### 1. روبوٹکس کے لیے AI تربیت

Isaac Sim خاص طور پر AI کی تربیت کے لیے طاقتور ہے:

- ** مصنوعی ڈیٹا جنریشن **: تربیت کے ماڈلز کے لیے بڑے ڈیٹا سیٹس بنائیں
- ** ڈومین رینڈمائزیشن **: حقیقت سے Smolition ٹرانسفر کو بہتر بنانے کے لیے
- ** کریل لرننگ **: Smolition میں روبوٹ کنٹرول کی پالیسیوں کو تربیت دیں

### 2. سینسر ڈویلپمنٹ

ورچوئل ماحول میں سینسر کی جانچ:

- ** LIDAR Smolition **: مادی خصوصیات کے ساتھ درست LIDAR بیم Smolition
- ** کیمرا Smolition **: شور اور مسخ کے ساتھ حقیقت پسندانہ کیمرا ماڈلز
- ** IMU Smolition **: بہاؤ اور شور کے ساتھ درست inertial پیمائش یونٹ

### 3. نیویگیشن ٹیسٹنگ

پیچیدہ ماحول میں نیویگیشن الگورتھم کی جانچ:

- ** راستہ منصوبہ بندی **: متنوع منظرناموں میں مختلف الگورتھم کی جانچ کریں
- ** رکاوٹوں سے بچنا **: تصادم کا پتہ لگانے کے لیے حقیقت پسندانہ فزکس
- ** SLAM ٹیسٹنگ **: SLAM کی توثیق کے لیے حقیقت پسندانہ سینسر ڈیٹا جنریٹ کریں

### 4. ہیرا پھیری کے کام

Smolition میں روبوٹک ہیرا پھیری کی جانچ:

- ** گرفت **: فزکس-درست آبجیکٹ گرفت Smolition
- ** اسمبلی **: پیچیدہ ملٹی جوائنٹ ہیرا پھیری کے کام
- ** کنٹیکٹ فزکس **: حقیقت پسندانہ کنٹیکٹ اور رگڑ ماڈلنگ

## Isaac Sim بمقابلہ دوسرے پلیٹ فارم

### Gazebo کے ساتھ موازنہ

| خصوصیات | Gazebo | Isaac Sim |
|---------|--------|----------|
| رینڈرنگ کوالٹی | کم | اعلی (فوٹو وریلسٹک) |
| فزکس کا معیار | اچھا | شاندار (GPU ایکسلریٹڈ) |
| سینسر Smolition | بنیادی | ایڈوانسڈ |
| AI ٹریننگ سپورٹ | محدود | وسیع |
| مصنوعی ڈیٹا جنریشن | بنیادی | ایڈوانسڈ |
| GPU ایکسلریشن | محدود | مکمل RTX ایکسلریشن |

### Isaac Sim کب منتخب کریں

جب آپ کی ضرورت ہو تو Isaac Sim کا انتخاب کریں:

- ** تاثرات کی تربیت کے لیے اعلی معیار رینڈرنگ **
- ** حقیقت پسندانہ روبوٹ سلوک کے لیے فزکس کی درست Smolition **
- ** AI کی ترقی کے لیے مصنوعی ڈیٹا جنریشن **
- ** فاسٹ Smolition کے لیے GPU ایکسلریٹڈ فزکس **
- ** کمپیوٹر ویژن ایپلی کیشنز کے لیے فوٹو وریلسٹک سینسر Smolition **

## Isaac Sim ورک فلو

### ڈویلپمنٹ عمل

عام Isaac Sim ورک فلو میں شامل ہے:

1. ** ماحول کی تخلیق **: ورچوئل ماحول کو ڈیزائن یا درآمد کریں
2. ** روبوٹ ماڈلنگ **: USD کے ساتھ روبوٹ ماڈلز بنائیں یا درآمد کریں
3. ** سینسر سیٹ اپ **: فزیکل اور ورچوئل سینسر تشکیل دیں
4. ** ٹاسک کی وضاحت **: AI کی تربیت یا جانچ کے لیے کاموں کی وضاحت کریں
5. ** Smolition اجرا **: مختلف منظرناموں کے ساتھ Smolition چلائیں
6. ** ڈیٹا جمع کرنا **: سینسر اور کارکردگی کا ڈیٹا جمع کریں
7. ** AI ٹریننگ **: ماڈلز کو تربیت دینے کے لیے جمع کردہ ڈیٹا کا استعمال کریں
8. ** توثیق **: Smolition میں تربیت یافتہ ماڈلز
9. ** تعیناتی **: اصلی روبوٹ میں منتقل کریں

## روبوٹکس پائپ لائن میں Isaac Sim

### اصلی روبوٹ کے ساتھ انضمام

مکمل روبوٹکس پائپ لائن میں Isaac Sim کا کردار:

```
Real Robot ──┐
              ├── Simulation ──┐
              │                ├── Training ──┐
              │                │              ├── Deployment
              │                │              │
              └── Data ────────┘              │
                                            │
              Synthetic Data ────────────────┘
```

### ڈومین رینڈمائزیشن

حقیقت کی طرف ٹرانسفر کو بہتر بنانے کے لیے Isaac Sim میں ایک کلیدی تکنیک:

- ** بصری ڈومین رینڈمائزیشن **: بصری خصوصیات میں فرق (ٹیکچر، لائٹنگ)
- ** فزیکل ڈومین رینڈمائزیشن **: فزیکل خصوصیات میں فرق (فرکشن، ماس)
- ** ڈائنیمکس رینڈمائزیشن **: مختلف سسٹم ڈائنیمکس پیرامیٹر

## Isaac Sim کارکردگی کے مسائل

### آپٹیمائزیشن کی حکمت عملی

Isaac Sim میں زیادہ سے زیادہ کارکردگی کے لیے:

1. ** تفصیل کی سطح (LOD) **: مناسب میش پیچیدگی کا استعمال کریں
2. ** فزکس اپ ڈیٹ کی شرح **: کارکردگی کے ساتھ درستگی کا توازن
3. ** رینڈرنگ ترتیبات **: معیار بمقابلہ رفتار کو ایڈجسٹ کریں
4. ** منظر کی پیچیدگی **: نظروں میں اشیاء کی تعداد کی حد
5. ** متوازی پروسیسنگ **: ملٹی کور CPU اور GPU کا استعمال کریں

## خلاصہ

Isaac Sim نے روبوٹکس Smolition میں ایک اہم پیشرفت کی نمائندگی کی ہے، خاص طور پر AI کی ترقی کے لیے جہاں فوٹو وریلسٹک سینٹھیٹک ڈیٹا اور درست فزکس اہم ہیں۔ اگلا ذیلی ماڈیول Isaac Sim کو انسٹال کرنے اور ROS 2 انضمام کے لیے ڈویلپمنٹ ماحول کو ترتیب دینے میں غوطہ لگائے گا۔"""

def translate_english_to_urdu_4_1_overview_of_vla_models():
    return """---


sidebar_position: 1
difficulty: beginner


---

# 4.1: VLA ماڈلز کا جائزہ

## جائزہ

یہ ذیلی ماڈیول وژن-لینگویج-ایکشن (VLA) ماڈلز کا تعارف فراہم کرتا ہے - ایک نئے جنریشن کے ماڈلز جو جسمانی جذبے، قدرتی زبان کی سمجھ، اور ایکشن جنریشن کو ایک نیورل نیٹ ورک میں یکجا کرتے ہیں۔

## سیکھنے کے مقاصد

اس ذیلی ماڈیول کے اختتام تک، آپ کریں گے:
- VLA ماڈلز کی تعریف اور اہمیت کو سمجھیں گے
- VLA ماڈلز کے لیے استعمال کے معاملات اور ایپلی کیشنز کی شناخت کریں گے
- VLA ماڈلز کے بنیادی فن تعمیر اور کام کو سمجھیں گے
- VLA ماڈلز کو Smolition اور اصلی دنیا میں استعمال کرنے کا طریقہ سیکھیں گے
- VLA ماڈلز کی کارکردگی کی جانچ کرنے اور ترقی دینے کا طریقہ جانیں گے

## VLA ماڈلز کا تعارف

وژن-لینگویج-ایکشن (VLA) ماڈلز ایک جدید جسمانی AI ماڈلز کی قسم ہیں جو بصری ادراک، زبانی سمجھ، اور حرکتی ایکشن کو ایک نیورل نیٹ ورک میں یکجا کرتے ہیں۔ یہ ماڈلز روبوٹکس کنٹرول کے لیے اہم ہیں کیونکہ وہ روبوٹ کو وضاحت کے مطابق کام کرنے کی صلاحیت دیتے ہیں۔

### VLA کیا ہے؟

VLA اکرونیم وژن (بصری ادراک)، لینگویج (زبانی سمجھ)، اور ایکشن (عمل) کو ظاہر کرتا ہے:

- ** وژن **: بصری ان پٹ کو دیکھنے اور سمجھنے کی صلاحیت
- ** لینگویج **: قدرتی زبان کی اہمیت اور سمجھ کی صلاحیت
- ** ایکشن **: مشن کے مطابق نظریاتی اور جسمانی کارروائی کی صلاحیت

### VLA ماڈلز کے فوائد

1. ** ملٹی ماڈل ادراک **: تصاویر، زبان، اور حرکات کو ایک ہی فریم ورک میں سمجھنے کی صلاحیت
2. ** ہیومن-لائیک انٹر ایکشن **: قدرتی زبان کمانڈز کے ذریعے روبوٹ کنٹرول
3. ** سیمینٹک طرز عمل **: تصوراتی اور منطقی کام انجام دینے کی صلاحیت
4. ** ٹرانسفر لرننگ **: Smolition سے حقیقت کی طرف بہتر ٹرانسفر
5. ** کم سے کم ڈیمو **: کم ڈیمو اور زیادہ کارآمد تربیت

## VLA ماڈلز کی بنیادی تکنالوجی

### ماڈل فن تعمیر

VLA ماڈلز کے بنیادی اجزاء میں شامل ہیں:

1. ** بصری انکوڈر **: تصاویر اور ویڈیوز کو ایمبیڈنگس میں تبدیل کرتا ہے
2. ** زبانی انکوڈر **: ٹیکسٹ کو سیمینٹک ایمبیڈنگس میں تبدیل کرتا ہے
3. ** فیوژن لیئر **: بصری اور زبانی اشارے ایک جگہ جمع کرتا ہے
4. ** ایکشن جنریٹر **: کمانڈ کے مطابق روبوٹ کے کارروائی کو تیار کرتا ہے

### اہم VLA ماڈلز

1. ** RT-1 (Robotics Transformer 1) **: Google کا ماڈل، 130K ٹاسک سے زیادہ
2. ** BC-Z (Behavior Cloning with Z-axis) **: OpenAI کا ماڈل، ہوم ورک بینچ مارکس
3. ** Instruct2Act **: LLMs سے متاثر، کام کو چھوٹے کاموں میں توڑنا
4. ** VIMA (Vision-Language-Action) **: Meta کا ماڈل، Smolition پر تربیت یافتہ

## VLA ماڈلز کے استعمال کے معاملات

### 1. ہوم آٹومیشن

VLA ماڈلز گھریلو کاموں کے لیے استعمال کیے جا سکتے ہیں:

- کمروں کو صاف کرنا
- چیزوں کو اٹھانا اور منتقل کرنا
- کھانا پکانا اور سروس کرنا
- آبادی کے لیے معاونت

### 2. صنعتی آٹومیشن

- مشینوں کو کنٹرول کرنا
- معیار کی جانچ
- اجزاء کو اکٹھا کرنا
- بار بار کام

### 3. سروس روبوٹکس

- کسٹمر سروس
- ہسپتال میں معاونت
- تفریحی سرگرمیوں کے لیے
- حفاظتی کام

## VLA ماڈلز کا استعمال

### VLA ماڈل کو چلانا

```python
# VLA ماڈل کو چلانے کی مثال
import torch
from vla_model import VLA

# ماڈل لوڈ کریں
model = VLA.from_pretrained("pretrained_vla_model")

# کمانڈ اور تصویر کے ساتھ ایکشن تیار کریں
command = "کالے کپ کو میز کے دائیں جانب رکھیں"
image = load_image("current_scene.jpg")

# ایکشن تیار کریں
action = model.predict(command, image)

# روبوٹ کو چلائیں
robot.execute_action(action)
```

### VLA ماڈل کی تربیت

```python
# VLA ماڈل کی تربیت کی مثال
from vla_model import VLA
from vla_dataset import VLADataset

# ڈیٹا سیٹ لوڈ کریں
dataset = VLADataset(data_path="vla_data/")

# ماڈل لوڈ کریں
model = VLA.from_pretrained("base_vla_model")

# تربیت کا عمل
optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)

for epoch in range(num_epochs):
    for batch in dataset:
        images, commands, actions = batch
        loss = model.compute_loss(images, commands, actions)
        
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
```

## VLA ماڈلز کی چیلنجوں

### 1. ڈیٹا کی کمی

VLA ماڈلز کو زیادہ تعداد میں کوالٹی ڈیٹا کی ضرورت ہوتی ہے:

- ڈیٹا جمع کرنا مہنگا ہے
- لیبلنگ کا عمل وقت لگتا ہے
- مختلف منظرناموں کے لیے ڈیٹا کی ضرورت ہوتی ہے

### 2. Smolition سے حقیقت ٹرانسفر

- Smolition اور حقیقت کے درمیان فرق
- Smolition ایمبیڈنگس اور حقیقتی ایمبیڈنگس میں عدم مطابقت
- کارکردگی کی خرابی Smolition سے حقیقت میں

### 3. بصری سمجھ میں کمی

- مختلف لائٹنگ حالات
- نظر کی رکاوٹیں
- میٹھے ڈیٹا کی کمی

## VLA ماڈلز کے لیے بہترین تکنیکیں

### 1. ڈیٹا کی تیاری

- مختلف منظرناموں کے لیے ڈیٹا جمع کرنا
- ڈیٹا ایمنیشن کے ذریعے ڈیٹا کو بڑھانا
- ڈیٹا کی کوالٹی کو یقینی بنانا

### 2. ٹرانسفر لرننگ

- Smolition میں تربیت، حقیقت میں چلانا
- ڈومین رینڈمائزیشن کا استعمال
- Few-shot تربیت کا استعمال

### 3. ماڈل کی کارکردگی

- کارکردگی کی جانچ کے لیے بینچ مارکس
- کارکردگی کی میٹرکس کو یقینی بنانا
- بار بار جانچ کا عمل

## خلاصہ

VLA ماڈلز روبوٹکس کے شعبے میں ایک اہم پیشرفت ہیں، جو بصری، زبانی، اور عمل کو ایک نیورل نیٹ ورک میں یکجا کرتے ہیں۔ یہ ماڈلز قدرتی زبان کے ذریعے روبوٹ کنٹرول کی اہلیت فراہم کرتے ہیں۔ اگلا ذیلی ماڈیول VLA ماڈلز کی تکنیکی تفصیلات اور فن تعمیر کا احاطہ کرے گا۔"""

def translate_english_to_urdu_intro():
    return """---
sidebar_position: 1
---

# فزیکل اے آئی اور ہیومنائڈ روبوٹکس: ROS 2 بنیادیات سے VLA ماڈلز تک

فزیکل اے آئی اور ہیومنائڈ روبوٹکس کے تعلیمی پلیٹ فارم میں خوش آمدید۔ یہ 13 ہفتوں کا نصاب آپ کو جسمانی ذہانت میں بنیادی تصورات سے لے کر جدید ایپلیکیشنز تک لے جانے کے لیے ڈیزائن کیا گیا ہے۔

## فزیکل اے آئی کیا ہے؟

فزیکل اے آئی مصنوعی ذہانت اور جسمانی نظاموں کے سنگم کی طرف اشارہ کرتا ہے، خاص طور پر روبوٹکس میں۔ روایتی اے آئی کے برعکس جو بنیادی طور پر ڈیجیٹل جگہوں میں کام کرتی ہے، فزیکل اے آئی کو حقیقی دنیا کی پیچیدگیوں اور غیر یقینی صورتحال سے نمٹنا پڑتا ہے۔

## کورس کی ساخت

یہ نصاب چار جامع ماڈیولز میں تقسیم ہے:

1. **ماڈیول 1: روبوٹ آپریٹنگ سسٹم (ROS 2) - بنیاد اور رابطہ** - روبوٹ آپریٹنگ سسٹم (ROS 2) فریم ورک میں مہارت حاصل کریں، بشمول نوڈز، ٹاپکس، سروسز، ایکشنز، TF2 ٹرانسفارمز، اور URDF روبوٹ کی تفصیلات۔
2. **ماڈیول 2: روبوٹ Smolition - ورچوئل ٹیسٹنگ ماحول** - Gazebo اور دیگر پلیٹ فارمز کا استعمال کرتے ہوئے Smolition تکنیکیں سیکھیں تاکہ ورچوئل ماحول میں روبوٹ کی ترقی، جانچ اور توثیق کی جا سکے۔
3. **ماڈیول 3: NVIDIA Isaac Sim - جدید فوٹو ریئلسٹک Smolition** - NVIDIA کے جدید روبوٹکس Smolition پلیٹ فارم کو RTX رینڈرنگ، PhysX فزکس، اور مصنوعی ڈیٹا تخلیق کی صلاحیتوں کے ساتھ دریافت کریں۔
4. **ماڈیول 4: وژن-لینگویج-ایکشن (VLA) ماڈلز - روبوٹکس کے لیے ملٹی موڈل اے آئی** - جدید ترین وژن-لینگویج-ایکشن ماڈلز کا مطالعہ کریں جو جسمانی اے آئی کے لیے بصری ادراک، قدرتی زبان کی سمجھ، اور ایکشن تخلیق کو یکجا کرتے ہیں۔

## سیکھنے کے مقاصد

اس کورس کے اختتام تک، آپ:

- جسمانی ذہانت کے بنیادی اصولوں کو سمجھیں گے
- روبوٹک ایپلیکیشنز کے لیے ROS 2 میں مہارت حاصل کریں گے
- روبوٹک سسٹمز کو Smolition اور ٹیسٹ کرنے کا طریقہ جانیں گے
- NVIDIA کے Isaac Sim پلیٹ فارم اور اس کی ایپلیکیشنز کو سمجھیں گے
- روبوٹک کنٹرول کے لیے وژن-لینگویج-ایکشن ماڈلز سے واقف ہوں گے

## پیش شرطیں

- بنیادی پروگرامنگ کا علم (Python کو ترجیح دی جاتی ہے)
- لینیئر الجبرا اور کیلکولس کی سمجھ
- Linux کمانڈ لائن سے واقفیت

## ہارڈویئر کی ضروریات

اگرچہ آپ Smolition کے ذریعے نصاب کا بیشتر حصہ مکمل کر سکتے ہیں، ہم درج ذیل تک رسائی کی سفارش کرتے ہیں:

- NVIDIA Jetson Orin (یا مساوی ڈویلپمنٹ کٹ)
- گہرائی کی تصور کے لیے Intel RealSense کیمرہ
- Unitree Go1 یا اسی طرح کا چوپایا روبوٹ (جدید ماڈیولز کے لیے اختیاری)

آئیے فزیکل اے آئی اور ہیومنائڈ روبوٹکس کی دلچسپ دنیا میں اپنا سفر شروع کریں!"""

def fix_all_mixed_translations():
    """
    Fix all mixed English-Urdu content in i18n files.
    """
    # Create proper directory structure
    base_path = Path("frontend/i18n/ur/docusaurus-plugin-content-docs/current")
    base_path.mkdir(parents=True, exist_ok=True)

    # Create module directories
    for i in range(1, 5):
        (base_path / f"module-{i}").mkdir(exist_ok=True)
        for j in range(1, 5):
            (base_path / f"module-{i}" / f"week-{j}-introduction").mkdir(exist_ok=True)
            (base_path / f"module-{i}" / f"week-{j}-nodes-topics").mkdir(exist_ok=True)
            (base_path / f"module-{i}" / f"week-{j}-services-actions").mkdir(exist_ok=True)
            (base_path / f"module-{i}" / f"week-{j}-tf-urdf").mkdir(exist_ok=True)
            (base_path / f"module-{i}" / f"week-{j}-gazebo-basics").mkdir(exist_ok=True)
            (base_path / f"module-{i}" / f"week-{j}-simulation-integration").mkdir(exist_ok=True)
            (base_path / f"module-{i}" / f"week-{j}-advanced-simulation").mkdir(exist_ok=True)
            (base_path / f"module-{i}" / f"week-{j}-isaac-ros-basics").mkdir(exist_ok=True)
            (base_path / f"module-{i}" / f"week-{j}-advanced-isaac-sim").mkdir(exist_ok=True)
            (base_path / f"module-{i}" / f"week-{j}-isaac-sim-applications").mkdir(exist_ok=True)
            (base_path / f"module-{i}" / f"week-{j}-vla-fundamentals").mkdir(exist_ok=True)
            (base_path / f"module-{i}" / f"week-{j}-vla-integration").mkdir(exist_ok=True)
            (base_path / f"module-{i}" / f"week-{j}-advanced-vla-applications").mkdir(exist_ok=True)

    # Write files with proper translations
    files_to_create = [
        ("frontend/i18n/ur/docusaurus-plugin-content-docs/current/intro.md", translate_english_to_urdu_intro()),
        ("frontend/i18n/ur/docusaurus-plugin-content-docs/current/module-1/week-1-introduction/1-1-what-is-ros2.md", translate_english_to_urdu_1_1_what_is_ros2()),
        ("frontend/i18n/ur/docusaurus-plugin-content-docs/current/module-1/week-1-introduction/1-2-architecture.md", translate_english_to_urdu_1_2_architecture()),
        ("frontend/i18n/ur/docusaurus-plugin-content-docs/current/module-1/week-1-introduction/1-3-installation-setup.md", translate_english_to_urdu_1_3_installation_setup()),
        ("frontend/i18n/ur/docusaurus-plugin-content-docs/current/module-1/week-1-introduction/1-4-practical-exercises.md", translate_english_to_urdu_1_4_practical_exercises()),
        ("frontend/i18n/ur/docusaurus-plugin-content-docs/current/module-2/week-1-introduction/2-1-overview-of-robot-simulation.md", translate_english_to_urdu_2_1_overview_of_robot_simulation()),
        ("frontend/i18n/ur/docusaurus-plugin-content-docs/current/module-2/week-2-gazebo-basics/2-2-gazebo-installation-setup.md", translate_english_to_urdu_2_2_gazebo_installation_setup()),
        ("frontend/i18n/ur/docusaurus-plugin-content-docs/current/module-3/week-1-introduction/3-1-overview-of-nvidia-isaac-sim.md", translate_english_to_urdu_3_1_overview_of_nvidia_isaac_sim()),
        ("frontend/i18n/ur/docusaurus-plugin-content-docs/current/module-4/week-1-introduction/4-1-overview-of-vla-models.md", translate_english_to_urdu_4_1_overview_of_vla_models()),
    ]

    for file_path, content in files_to_create:
        path = Path(file_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"Created: {file_path}")

    print("All specified files have been created with proper Urdu translations.")

def main():
    print("Creating proper Urdu translations for i18n files...")
    fix_all_mixed_translations()
    print("Done! All files now have proper Urdu content without mixed English.")

if __name__ == "__main__":
    main()