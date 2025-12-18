---


sidebar_position: 3
difficulty: beginner


---
# منبر 3: سروسس اوسن

## ج a ج

ہف پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ نیچے پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ نیچے پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ یچے یچے یچے ا ٹ ٹ ٹ ی ٹ ی ی ی ی ی ی ی ڈیکپ ڈیکپ ڈیکپ ڈیکپ ڈیکپ ڈیکپ ڈیکپ ڈیکپ ڈیکپ ڈیکپ ڈیکپ ڈیکپ ڈیکپ ڈیکپ ڈیکپ ڈیکپ ی ی ی ی ی ی ی ی

## ss یکھ n ے کے maua ص d

کے ذ ک ک ک ک a/کی کی خ خ خ خ atatam ک a یہ یہ ہف ہف آپ آپ ک ک ک ک ک ک ک ک ے ے ے ے گ گ گ گ گ
- آsas/کی ٹ asas ، sross ز ، ، ، ، ، ، ، ، کے کے کے کے کے کے کے کے کے کے کے
۔
- ROS 2 A یکش N ز
- ک a/کی mnassb mwa ص Lat پیٹ پیٹ l یے l یے l یے l یے mautli Jatmalat کے maaaamlat ک a anatahab ک
- Debug سروس اور ایکشن مواصلات

## سروسس ، آ روس 2

srsross sidri خ خ خ خ ک ک ک ک ک کے کی کی کی کی کی ک ج ج ک ک ک ک کی کی کی ک ک ک ک ک ک ک ک کی کی ک ک ک ک ک ک ک ک ک کی ک ک ک ک ک ک ک ک ک ک یک یک ک ک ک ک ک ک ک ک ک یک یک یک یک یک کی کی کی کی کی کی کی کی کی کی کی کی ہے ہے ہ ہ ہ ہ M Waut ssasiک mwolat۔

### سروس تعر یف
سروس definitions ہیں stored میں `.srv` files کے ساتھ دو parts: کا/کی request اور کا/کی response, separated کے ذریعے تین dashes (`---`).
مثال: `AddTwoInts.srv`
```
int64 ایک
int64 b
---
int64 sum
```
### سروس سرور
```python
import rclpy
سے rclpy.نود import نود
سے example_interfaces.srv import AddTwoInts

class MinimalService:
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.ایک + request.b
        self.get_logger().info
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
### -srws ک Laaun ٹ
```python
import rclpy
سے rclpy.نود import نود
سے example_interfaces.srv import AddTwoInts

class MinimalClient:
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        جب تک نہیں self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info
        self.req = AddTwoInts.Request()

    def send_request:
        self.req.ایک = ایک
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(41, 1)
    minimal_client.get_logger().info
    minimal_client.destroy_node()
    rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## اعنی ی ی ز maa ئی revs 2

ک ک ک ک ک ک ِ ِ ِ ِ ِ ِ ِ ِ ِ ِ ِ ِ ِ ِ آ آ آ/آ raa کے/نعتا کے junaumon ہ کی کی rvau ک rvau ک rati ے

### a (تعر یف
ایکشن definitions ہیں stored میں `.ایکشن` files کے ساتھ تین parts: Goal, Result, اور Feedback.
مثال: `Fibonacci.ایکشن`
```
int32 order
---
int32[] sequence
---
int32[] sequence
```
### a یش nsrur
```python
import rclpy
سے rclpy.ایکشن import ActionServer, CancelResponse, GoalResponse
سے rclpy.نود import نود
سے example_interfaces.ایکشن import Fibonacci

class MinimalActionServer:
    def __init__(self):
        super().__init__('minimal_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        کے لیے میں میں range(1, goal_handle.request.order):
            اگر goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info
                return Fibonacci.Result()
            
            feedback_msg.sequence.append
            self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Returning result: {result.sequence}')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = MinimalActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
### asn ک laiun ٹ
```python
import rclpy
سے rclpy.ایکشن import ActionClient
سے rclpy.نود import نود
سے example_interfaces.ایکشن import Fibonacci

class MinimalActionClient:
    def __init__(self):
        super().__init__('minimal_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        اگر نہیں goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = MinimalActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)

اگر __name__ == '__main__':
    main()
```
## ک a/کی صحیح mwaulat nonunmonہ ک a anata خ ab intahna

| آرن | کی s کی کی کی ک خص خص خص خص
| --------- | ---------- | ----------------------- |
| ٹ a پک s | Mastی ڈیٹ a ڈیٹ a asas ٹ raum ز | غی r متال ز ڈیکپ ڈیکپ l ڈ |
| سروسس | آ سان ڈاری خ واسات/ج WABO | ہM وِت ساس ، ، ، ، ، ، ، ، ، ک ک
| a یکش n ز | لامبا- اللنی ے ک ک Am | غی غی غی کے کے کے کے کے کے SAAT ھ ھ ھ ھ ھ ھ ھ آ vr کی Mnnsov خی |

### ک B ک B ہ R جونی ہ JASTAAMAIL ALSR یں

.
.
.

## عمل ، ممامال: روبوبی ی ی ی ی ی ی ک ک ک ک ک ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی

آئیے آئیے ہ ہ ہ ہ ہ آئیے آئیے آئیے آئیے آئیے آئیے آئیے آئیے آئیے:
```python
# روبوٹ Arm Controller
import rclpy
سے rclpy.نود import نود
سے rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
سے std_msgs.msg import Float32MultiArray
سے example_interfaces.srv import SetBool
سے example_interfaces.ایکشن import MultiGoal
سے rclpy.ایکشن import ActionServer, GoalResponse

class RobotArmController:
    def __init__(self):
        super().__init__('robot_arm_controller')
        
        # ٹاپک: Joint angles feedback
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.joint_publisher = self.create_publisher(Float32MultiArray, 'joint_angles', qos_profile)
        
        # سروس: Check اگر arm ہے ready
        self.ready_service = self.create_service(SetBool, 'arm_ready', self.arm_ready_callback)
        
        # ایکشن: Move کو position
        self.move_action_server = ActionServer(
            self,
            MultiGoal,
            'move_arm',
            self.execute_move_callback
        )
        
        # Simulate joint angles
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 joints
        self.timer = self.create_timer(0.1, self.publish_joint_angles)
    
    def publish_joint_angles(self):
        msg = Float32MultiArray()
        msg.data = self.joint_angles
        self.joint_publisher.publish(msg)
    
    def arm_ready_callback(self, request, response):
        # Check اگر تمام joints ہیں within safe limits
        is_safe = تمام
        response.success = is_safe
        response.message = f'Arm ہے {"ready" اگر is_safe else "نہیں ready"}'
        return response
    
    def execute_move_callback(self, goal_handle):
        self.get_logger().info
        # Simulate movement
        self.joint_angles = goal_handle.request.goals
        goal_handle.succeed()
        result = MultiGoal.Result()
        result.result = True
        return result

def main(args=None):
    rclpy.init(args=args)
    controller = RobotArmController()
    rclpy.spin(controller)
    rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## سروس اوسن اوسون

موڈ لامان-لائیون ٹ اولی کے کے ہی ہی ہی ک ک ک am ک am arir ک ہیں SAATI srsrosi ز Awr Adun ز:
```bash
# سروسز
ros2 سروس list                    # List تمام سروسز
ros2 سروس info /service_name      # Get information about ایک سروس
ros2 سروس type /service_name      # Get سروس type
ros2 سروس call /service_name service_type "request_data"

# ایکشنز
ros2 ایکشن list                     # List تمام ایکشنز
ros2 ایکشن info /action_name        # Get information about ایک ایکشن
ros2 ایکشن send_goal /action_name action_type "goal_data"
```
## babٹbr ی ط r یقہ ک ari

1. سروسس کے کے کے کے کے کے کے quick quish quick quick quick quick quick quick quick quick quick quick quick quick quick quick quick quick quick quick quick quick quick فوری
2. یسمال العب العلوبی- اللان ک وولے ا السریشنییز _ _ _ _ _ _ _ _ ک ک ک ک کی کی کی کی ہے _
3.
4. ہ میک سرروز آسون ک لاؤون ٹ کی غ غ غ Lauchoi ک v juna ڈ l -acr یں
5. مناس بی ٹ aa ئ m آؤٹ ک ک v jna فذ ک r یں کے کے کے

## خ LAA صہ

ہف ک ک ک ک ک ک ک ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ک ک ​​ک ک ک ز ز ز ز ک ک ​​ک ک ک ک ز ز ز ک ک ​​ک ک ک ک ز ز ز ز ز ک ک ​​ک ک ک ز ز ز ز ز ز ز ک ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ہف ہف ہف ہف ہف ہف ہف ہف ہف ہف نیچے ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز نیچے ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ز ہے ہے ز ز ز ز ز ز ز ز ز ز آپ ساسا ہوا کی s ے ک ک v لیمل مول مول مول ڈونواؤ جونواچ جونوچ ں ک a asatamail ، ، آ آ آ آ ک ک ک ک ک ک آ آ آ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ 

ِsla ، ہ ہ ہ ہ ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹی ٹ ٹ ٹ ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض maa ڈ ln گ کی کی t lai ک ک یں گے۔ گے۔

## ماؤس

؛
2
3. تعمگر ک r یں
4.
