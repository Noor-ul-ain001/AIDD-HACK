---


sidebar_position: 3
difficulty: intermediate


---

# 3.3: مواصلاتی پیٹرنس کا موازنہ - ٹاپکس، سروسز، اور ایکشنز

## جائزہ

یہ سب مodule مختلف مواصلاتی پیٹرنس کا موازنہ کرتا ہے: ٹاپکس، سروسز، اور ایکشنز، جن کا استعمال ROS 2 میں مختلف مواصلاتی ضروریات کو پورا کرنے کے لیے کیا جاتا ہے۔

## سیکھنے کے مقاصد

اس ذیلی ماڈیول کے اختتام تک، آپ کریں گے:
- ٹاپکس، سروسز، اور ایکشنز کے فوائد کو سمجھیں گے
- مختلف مواصلاتی پیٹرنس کے درمیان توازن کو سمجھیں گے
- مناسب مواصلاتی پیٹرن کا انتخاب کریں گے
- مشترکہ مسائل سے متعلق خرابیوں کا ازالہ کریں گے

## مواصلاتی پیٹرنس کا خلاصہ

### 1. ٹاپکس

** فوائد: **
- غیر متزامن ڈیٹا کی ترسیل
- متعدد شائع کنندگان اور میزبانوں
- ٹاپک اور جوابی سرگرمیوں کے درمیان غیر منحصر ہونا
- حقیقی وقت کے ڈیٹا کی ترسیل
- سسٹم کو بوجھ سے بچنے کے لیے متعدد میزبانوں کی اجازت

** محدودیتیں: **
- غیر مستحکم ترسیل
- ڈیٹا کے نقصان کا امکان
- ترسیل کی تصدیق نہیں ہے
- کوئی ردعمل نہیں
- سسٹم کے وسائل پر اضافی بوجھ

### 2. سروسز

** فوائد: **
- ہم آہنگ معلومات کا تبادلہ
- درخواست اور جواب کی یقین دہانی
- کم وقت کی تاخیر
- ڈیٹا کی ترسیل کی تصدیق

** محدودیتیں: **
- بلاکنگ کالز ہو سکتی ہے
- تاخیر کی اجازت نہیں ہے
- ایک وقت میں صرف ایک کلائنٹ اور سرور کے درمیان بات چیت

### 3. ایکشنز

** فوائد: **
- طویل المدتہ کاموں کے لیے مناسب
- ریل ٹائم فیڈ بیک کی اجازت
- منسوخ کرنے کی صلاحیت
- کام کی پیشرفت کی مانیٹر کرنا
- غیر متزامن کام انجام دینے کی صلاحیت

** محدودیتیں: **
- زیادہ پیچیدہ نفاذ
- زیادہ وسائل کا استعمال
- زیادہ نیٹ ورک اور وسائل کی کھپت

## مواصلاتی پیٹرنس - مختصر خلاصہ

| خصوصیت | ٹاپک | سروس | ایکشن |
| --------- | ------- | --------- | -------- |
| ** طرز ** | اشتراک | درخواست/جواب | طویل المدتہ کام |
| ** ہم آہنگی ** | غیر ہم آہنگ | ہم آہنگ | ہم آہنگ (البتہ غیر مسلسل) |
| ** ترسیل ** | امکانی نقصان | یقینی | یقینی |
| ** دورانیہ ** | مسلسل | مختصر | طویل |
| ** تصدیق ** | نہیں ہے | ہے | ہے |
| ** منسوخی ** | نہیں | نہیں | ہے |
| ** استعمال کی صورتیں ** | سینسر ڈیٹا، ریاست کے اشارے | تشکیل، سادہ متغیرات | نیویگیشن، روبوٹ کے کام |

## کیس مطالعہ: ہر پیٹرن کا استعمال

### 1. سینسر ڈیٹا کا استعمال

** موقع: **

- متواتر سینسر ڈیٹا کا اشتراک
- میزبانوں کی بڑی تعداد
- چھوٹی تاخیر کی اجازت
- ڈیٹا کے نقصان کی چھوٹی اجازت

** عملی مثال: **

```python
# ٹاپک کے لیے مستقل سینسر ڈیٹا
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(LaserScan, 'laser_scan', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = LaserScan()
        # سینسر ڈیٹا کو اس میں شامل کریں
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()
```

** کب استعمال کریں: **

- سینسر ڈیٹا کی مستقل اسٹریم
- ریل ٹائم سٹیٹس اپ ڈیٹس
- متعدد subscribers کو ڈیٹا کی ترسیل
- چھوٹے ڈیٹا کی اپ ڈیٹس کے لیے جہاں تھوڑا ڈیٹا نقصان قابل قبول ہو
- کم تاخیر کے ساتھ کمیونیکیشن

### 2. سروس کا استعمال

** موقع: **

- فوری جواب کی ضرورت
- ایک وقت میں ایک کلائنٹ اور سرور کے تعلقات
- یقین دہانی کے ساتھ ڈیٹا کی ترسیل
- چھوٹے آپریشنز کے لیے جلد از جلد نتیجہ ضروری ہو

** عملی مثال: **

```python
# سرور کا مثال
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

** کب استعمال کریں: **

- تبدیلی کی درخواست
- ڈیٹا کی فوری بازیافت
- کنٹرول کے فیصلے
- واحد ٹرانزیکشنز
- ایک جیسے سوالات کے جوابات

### 3. ایکشن کا استعمال

** موقع: **

- طویل المدتہ کاموں کے لیے
- کام کی پیشرفت کی رپورٹنگ
- منسوخ کرنے کی ضرورت
- کام کے نتیجے کی اہمیت

** عملی مثال: **

```python
# ایکشن کا مثال
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()
```

** کب استعمال کریں: **

- نیویگیشن کے کام
- لمبے کاموں کے لیے جیسے کہ کنٹرول یا ہیرا پھیرا
- کام کی پیشرفت کی ضرورت
- کام کو منسوخ کرنے کی ضرورت
- کام کے نتائج کی اہمیت

## کون سا پیٹرن منتخب کریں؟ فیصلہ کا گراف

```
START: ضرورت ہے کام کے لیے مواصلات؟
│
├─ ضرورت ہے فوری جواب؟ ──┐
│                              │
├─ ہاں ──┐                    │
│         │                    │
│         ├─ چھوٹا آپریشن؟ ──┤
│         │                   │
│         ├─ ہاں ─ سروس       │
│         │                   │
│         └─ نہیں ──┐         │
│                  │         │
│                  ├─ منسوخ کر سکتے ہیں؟ ──┐
│                  │                       │
│                  ├─ ہاں ─ ایکشن         │
│                  │                       │
│                  └─ نہیں ─ سروس          │
│                                          │
└─ نہیں ──┐                                 │
         │                                 │
         ├─ جاری اسٹریم ڈیٹا؟ ────┤
         │                        │
         ├─ ہاں ─ ٹاپک          │
         │                        │
         └─ نہیں ──┐              │
                  │              │
                  ├─ طویل کام؟ ──┐
                  │               │
                  ├─ ہاں ──┐     │
                  │         │     │
                  │         ├─ منسوخ کر سکتے ہیں؟ ──┐
                  │         │                       │
                  │         ├─ ہاں ─ ایکشن         │
                  │         │                       │
                  │         └─ نہیں ─ سروس          │
                  │                                 │
                  └─ نہیں ──┐                        │
                           │                        │
                           └─ چھوٹا ─ سروس         │
                                         │
                                         └─ ضرورت ہے فیڈ بیک؟ ──┐
                                                                 │
                                                                 ├─ ہاں ─ ایکشن
                                                                 │
                                                                 └─ نہیں ─ سروس
```

## عملی مثالیں اور استعمال کے مواقع

### سینسر ڈیٹا اسٹریم کا استعمال

```python
# ٹاپکس کے لیے جاری سینسر ڈیٹا
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Odometry

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # متعدد سینسرز کے لیے publishers
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.laser_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # ڈیٹا کو باقاعدگی سے publish کرنے کے لیے ٹائمر
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

    def publish_sensor_data(self):
        # سینسر ڈیٹا کو publish کریں
        imu_msg = Imu()
        laser_msg = LaserScan()
        odom_msg = Odometry()
        
        self.imu_publisher.publish(imu_msg)
        self.laser_publisher.publish(laser_msg)
        self.odom_publisher.publish(odom_msg)

# یہ مناسب ہے کیونکہ:
# - سینسر ڈیٹا مستقل ہے
# - متعدد subscribers ڈیٹا چاہتے ہیں
# - چھوٹی تاخیر قابل قبول ہے
# - ڈیٹا کا کچھ نقصان قابل قبول ہے
```

### روبوٹ تشکیل کے لیے سروس

```python
# روبوٹ تشکیل کے لیے سروس
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters

class RobotConfigService(Node):
    def __init__(self):
        super().__init__('robot_config_service')
        self.config_service = self.create_service(
            SetParameters, 
            'set_robot_config', 
            self.set_config_callback)

    def set_config_callback(self, request, response):
        # تشکیل کی تصدیق اور اطلاق
        try:
            # نئی تشکیل کا اطلاق
            self.apply_configuration(request.parameters)
            response.successful = True
            response.result.message = "Configuration applied successfully"
        except Exception as e:
            response.successful = False
            response.result.message = f"Failed to apply configuration: {str(e)}"

        return response

# یہ مناسب ہے کیونکہ:
# - تشکیل کی تصدیق فوراً چاہیے
# - کلائنٹ کو کامیابی یا ناکامی کی تصدیق چاہیے
# - آپریشن چھوٹا ہے
# - قابل اعتماد ترسیل کی ضرورت ہے (سروس QoS)
```

### نیویگیشن ایکشن

```python
# نیویگیشن کے لیے ایکشن
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self.nav_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_navigation,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def execute_navigation(self, goal_handle):
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        # نیویگیشن کریں جس میں فیڈ بیک اور منسوخ کرنے کی صلاحیت ہو
        for step in range(100):  # سادہ نیویگیشن
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.result = -1  # CANCELED
                return result

            # نیویگیشن کی پیشرفت کو اپ ڈیٹ کریں
            feedback_msg.current_pose = self.get_current_pose()
            feedback_msg.distance_remaining = self.calculate_distance_remaining()

            goal_handle.publish_feedback(feedback_msg)

            # گول کی طرف جائیں (سادہ)
            import time
            time.sleep(0.1)

        goal_handle.succeed()
        result.result = 1  # SUCCESS
        return result

# یہ مناسب ہے کیونکہ:
# - نیویگیشن طویل چلنے والا کام ہے
# - پیشرفت کی فیڈ بیک قیمتی ہے
# - کام کو منسوخ کرنے کی ضرورت ہو سکتی ہے
# - نتیجہ مکمل ہونے کے بعد اہم ہے
```

## کارکردگی کے انتہائی مسائل

### ٹاپک کارکردگی کے مسائل

** فوائد: **
- زیادہ کارکردگی والے ڈیٹا ٹرنس میشن
- کم لیٹنسی
- سرور لوڈ کو کم کرنے کے بغیر متعدد subscribers
- ڈیکوپلڈ مواصلات

** محدودیتیں: **
- تصدیق کی عدم موجودگی
- ڈیٹا کا نقصان ممکن ہے
- میموری استعمال پر مسلسل دباؤ
- بڑے پیغامات کی اجازت نہیں

```python
# بہترین ٹاپکس کی اشاعت
class OptimizedSensorPublisher(Node):
    def __init__(self):
        super().__init__('optimized_sensor_publisher')

        # مختلف ڈیٹا کی اقسام کے لیے مناسب QoS استعمال کریں
        self.critical_qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        self.debug_qos = rclpy.qos.QoSProfile(
            depth=1,  # میموری بچانے کے لیے بُعد کو محدود کریں
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        self.critical_publisher = self.create_publisher(CriticalData, 'critical', self.critical_qos)
        self.debug_publisher = self.create_publisher(DebugData, 'debug', self.debug_qos)
```

### سروس کارکردگی کے مسائل

** فوائد: **
- یقینی طور پر ڈیٹا کی ترسیل
- ہم آہنگ جواب
- آسان ڈیبگنگ اور ٹیسٹنگ

** محدودیتیں: **
- بلاکنگ کالز ہو سکتی ہے
- امکانی سرور کی رکاوٹیں
```python
# ٹائم آؤٹ کے ساتھ بہترین سروس کلائنٹ
class OptimizedServiceClient(Node):
    def __init__(self):
        super().__init__('optimized_service_client')
        self.client = self.create_client(SetParameters, 'set_parameters')

    async def call_service_with_timeout(self, req, timeout=5.0):
        # چیک کریں کہ سروس دستیاب ہے
        if not self.client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('Service not available')
            return None

        # Async کال کریں تاکہ بلاک نہ ہو
        future = self.client.call_async(req)

        # نتیجہ کے لیے انتظار کریں اور ٹائم آؤٹ کے ساتھ
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if future.done():
            return future.result()
        else:
            self.get_logger().error('Service call timed out')
            return None
```

### ایکشن کارکردگی کے مسائل

** فوائد: **
- طویل المدتہ کاموں کی نگرانی
- منسوخ کرنے کی صلاحیت
- مناسب کے لیے طویل چلنے والے کام
- نتیجہ کی فراہمی کی اہمیت

** محدودیتیں: **
- زیادہ پیچیدہ نفاذ
- زیادہ وسائل کا استعمال
- زیادہ نیٹ ورک اوور ہیڈ

```python
# مناسب فیڈ بیک کے ساتھ بہترین ایکشن
class OptimizedActionServer(Node):
    def __init__(self):
        super().__init__('optimized_action_server')
        self.action_server = ActionServer(
            self,
            LongRunningTask,
            'long_task',
            execute_callback=self.execute_long_task,
            goal_callback=self.goal_callback)

    def execute_long_task(self, goal_handle):
        start_time = self.get_clock().now()
        feedback_msg = LongRunningTask.Feedback()
        result = LongRunningTask.Result()

        # مناسب فیڈ بیک وقفہ کا حساب
        total_steps = goal_handle.request.steps
        feedback_interval = max(1, total_steps // 50)  # زیادہ سے زیادہ 50 فیڈ بیک پیغامات

        for i in range(total_steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.completion_percentage = (i / total_steps) * 100.0
                return result

            # اصل کام انجام دیں
            self.perform_work_step(i)

            # حساب کے مطابق فیڈ بیک بھیجیں
            if i % feedback_interval == 0:
                feedback_msg.completion_percentage = (i / total_steps) * 100.0
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                feedback_msg.estimated_time_remaining = (elapsed / (i + 1)) * (total_steps - i)

                goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result.completion_percentage = 100.0
        return result
```

## عام غلطیاں اور بہترین مشقیں

### 1. ٹاپک کے غلط استعمال کی اقسام

```python
# غلط: ٹاپکس کے لیے کنفیگریشن کے لیے استعمال کرنا
config_publisher.publish(ConfigurationRequest())  # جواب کی ضمانت نہیں ہے

# اچھا: کنفیگریشن کے لیے سروسز استعمال کریں
future = config_client.call_async(config_request)

# غلط: بڑے پیغامات کو بار بار publish کرنا
# point clouds کو 30Hz پر ٹاپک پر

# اچھا: مناسب فریکوєنسی اور پیغام کا سائز استعمال کریں
# یا بڑے ڈیٹا کے لیے کمپریشن/ڈیسیمیشن استعمال کریں
```

### 2. سروس کے غلط استعمال کی اقسام

```python
# غلط: سیensor ڈیٹا اسٹریمنگ کے لیے سروسز استعمال کرنا
# ہر درخواست کے لیے سرور کو بلاک کرے گا

# اچھا: مستقل ڈیٹا کے لیے ٹاپکس استعمال کریں
sensor_publisher.publish(sensor_data)

# غلط: طویل چلنے والے کاموں کے لیے سروسز استعمال کرنا
# کلائنٹ کو ممکنہ طور پر منٹس کے لیے بلاک کرے گا

# اچھا: طویل کاموں کے لیے ایکشنز استعمال کریں
navigation_goal = NavigateToPose.Goal()
action_client.send_goal(navigation_goal)
```

### 3. ایکشن کے غلط استعمال کی اقسام

```python
# غلط: سادہ کوائف کے لیے ایکشنز استعمال کرنا
# سادہ آپریشنز کے لیے بہت پیچیدہ ہو سکتا ہے

# اچھا: سادہ کوائف کے لیے سروسز استعمال کریں
response = simple_query_service.call_async(query_request)

# غلط: ایسے مقاصد کے لیے جو مستقل حالت ہونی چاہیے
# جیسے کہ روبوٹ کی موجودہ پوزیشن

# اچھا: گول-اورینٹڈ کاموں کے لیے ایکشنز استعمال کریں
action_client.send_goal(navigate_goal)
```

## مثالیں: کیسے ٹرنس فارم کریں

### بہتر نمونہ ٹرنس فارم: ٹاپک سے سروس (جب ضمانت کی ضرورت ہو)

```python
# پہلے: تشکیل کی اشاعت
config_publisher.publish(new_config)

# بعد میں: ڈلیوری کی ضمانت کے لیے سروس استعمال کریں
future = config_service.call_async(config_request)

# مثال 2: سروس سے ایکشن
# پہلے: طویل وقت تک بلاک ہونے والی سروس
response = navigation_service.call_async(nav_request)  # منٹس تک بلاک ہو سکتا ہے

# بعد میں: پیشرفت کی فیڈ بیک کے ساتھ ایکشن استعمال کریں
action_client.send_goal(nav_action_goal)  # غیر بلاکنگ، فیڈ بیک کے ساتھ
```

## استنتاج اور سبق

### کب کون سا استعمال کریں:

1. ** ٹاپکس **:
   - جاری ڈیٹا اسٹریم کے لیے (سینسرز، ریاست کے اشارے)
   - متعدد subscribers کے لیے ڈیٹا کی ترسیل
   - کم تاخیر کی ضرورت ہو
   - کچھ پیغام کے نقصان کی اجازت ہو
   - اسٹیٹلیس مواصلات چاہیے

2. ** سروسز **:
   - یقین دہانی والی ڈیٹا کی ترسیل
   - فوراً کام کی انجام دہی چاہیے (1سے کم)
   - کنفیگریشن یا اہم ڈیٹا کی تبدیلی
   - مسلسل تبدیلیوں کی انجام دہی
   - سادہ سوالات کے جوابات

3. ** ایکشنز **:
   - طویل کاموں کے لیے (1سے زیادہ)
   - کام کی پیشرفت کی قیمتی ہو
   - کام کے دوران منسوخی کی ضرورت ہو
   - کام کا نتیجہ اہم ہو
   - کام کو متعدد مراحل میں ڈیزائن کریں

### عملی مثال: مکمل سسٹم

```python
# مثال: تمام پیٹرنس کا مناسب طریقے سے استعمال
class RoboticSystem(Node):
    def __init__(self):
        super().__init__('robotic_system')

        # ٹاپکس: مستقل سینسر/ریاست کی معلومات
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.status_pub = self.create_publisher(Status, 'robot_status', 10)

        # سروسز: کنفیگریشن اور فوری سوالات
        self.config_srv = self.create_service(SetConfig, 'set_config', self.config_callback)
        self.query_srv = self.create_service(GetState, 'get_state', self.state_callback)

        # ایکشنز: طویل کام
        self.nav_server = ActionServer(self, NavigateToPose, 'navigate', self.nav_execute)
        self.arm_server = ActionServer(self, MoveArm, 'move_arm', self.arm_execute)
```

## خلاصہ

یہ سب مodule مختلف مواصلاتی پیٹرنس - ٹاپکس، سروسز، اور ایکشنز - کا ایک جامع خلاصہ فراہم کرتا ہے۔ ہر ایک کے فوائد، محدودیتیں، اور مناسب استعمال کی صورتیں، اور جب کون سا استعمال کریں، اس کی وضاحت کرتا ہے۔ ٹاپکس مستقل ڈیٹا کے لیے، سروسز فوری طور پر کام کے لیے، اور ایکشنز طویل کاموں کے لیے مناسب ہیں۔ مناسب پیٹرن کا انتخاب کارکردگی اور نظام کی مربوطی کو بہتر بناتا ہے۔