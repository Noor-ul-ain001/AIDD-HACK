---


sidebar_position: 2
difficulty: beginner


---
# 3.2: ros 2 a یکش n ز ک ک ​​ک ک ک ک ک ک ک ک ک ک ک ک ک

## ج a ج

یہ ی ی s ی ی ی ی ی ی s ی s ی s ی s ی s ی s ی ی s ی s ی ی s ی ی s ی ی ی ی s ی ی s ی ی s ی ی ی ی s ی ی s ی s ی ی ی ی ی s ی ی ی s ی ی ی ی ی ی ی ی ی ی ی s ی ی ی ی ی ی ی ی ی s ی ی ی ی ی ی ی ی ی s ی ی ی ی ج ی ی ی ی s ی ی ی ی ج ی ی ی ی s ی ی ی ی ج ی ی ی ی s ی ی ی موولاسٹ جووالاسٹ جووالاسٹ الاؤولول ----- ز دونوک اوستا ک ک a ک a ک a ک a ک a j junwau srsross osswr۔

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- شmw کہ
- - ایسور اِسنا فذ اعاریں اِن سرور
- کام کے ssaٹ ھ bulٹ- maus آvrss ک si ٹ m aiaun asasnasn
- موڈ ک v snanbauln ے کے SAAT ھ آ RAA ء کے کے کے ن ن O ک V JNA فذ ک RI
- - اسسا ک asamamal ک SA
- مناسب استعمال کے معاملات کا اطلاق کریں

## اعان موولات بعدق

ROS 2 a یکش n ز پ پ پ پ پ پ د د ک ک ک ک پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ کے saati کے saati ھ saati ھ ھ ک ک ک ک/کی منڈرجہ

- **Goal**: Request کو perform ایک لمبا-running task
.
.
.

### ک B ک S SASATAMAL ک R یں

| a یکش n ز ک b | estasamal - srwsss ک biی | یسعمال ک یں پک s ک Be |
| -------------------------- | ----------------------- | ----------------------- |
| لامبا- اللنی ے ک ک Am | ف vr ی ی wab کی ض rorrat | مسالسال |
| stہ کی کی ض ض ض ض ض ض کے کے کے کے کے کے کے کے کے کے کے کے کے کے آ آ آ آ آ آ آ خ vasat/ج wabo | غی ر متالیل نِن سکرت |
| ک امواوم ماؤٹ ٹاسل ز | سنسر یsasasmid |
| گ اللو پ r Mubn ی کام | توو چیک | raausast ی jaaaaahat |
| منوسو آپ آپ r یش n | یک- ٹ aa ئ m پ rosasassn گ | at حیثی کی کی کی tt at ar ی |

## ایشن ڈھ چہ چہ چہ چہ ڈھ ڈھ ڈھ ڈھ ڈھ ڈھ

ہ ہ ق ق ق sm ق Sm کے jrauta ہے tal ز mausus ج کی کی کی کی کی کی کی
1.
2
3
مثال: `Fibonacci.ایکشن`
```
# Goal definition
int32 order
---
# Result definition
int32[] sequence
---
# Feedback definition
int32[] sequence
```
## اِن سرورز تال دِک ہے ہیں

### بن ی AAD ی سرور
```python
import time
import rclpy
سے rclpy.ایکشن import ActionServer, CancelResponse, GoalResponse
سے rclpy.نود import نود
سے example_interfaces.ایکشن import Fibonacci

class FibonacciActionServer:
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        """Accept یا reject ایک client request کو begin ایک ایکشن."""
        self.get_logger().info('Received goal request')
        # Accept تمام goals کے لیے یہ مثال
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept یا reject ایک client request کو cancel ایک ایکشن."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute کا/کی goal اور provide feedback."""
        self.get_logger().info('Executing goal...')
        
        # Create feedback message
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        # Simulate لمبا-running task
        کے لیے میں میں range(1, goal_handle.request.order):
            اگر goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Update feedback
            feedback_msg.sequence.append(
                feedback_msg.sequence[میں] + feedback_msg.sequence[میں-1])
            
            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            
            # Sleep کو simulate work
            time.sleep(1)

        # Check اگر we تھے canceled
        اگر goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return Fibonacci.Result()

        # Success
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = FibonacciActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()
```
## یکش یکش ک ک ک laa ئ n ٹ s ک ک v tacl ے re ہے ہیں ہیں

### bin ی aad ی ک laa ئ n ٹ
```python
import time
import rclpy
سے rclpy.ایکشن import ActionClient
سے rclpy.نود import نود
سے example_interfaces.ایکشن import Fibonacci

class FibonacciActionClient:
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        # Wait کے لیے ایکشن server
        self._action_client.wait_for_server()
        
        # Create goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        
        # Send goal اور get future
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        # Add callback کے لیے کب goal ہے accepted
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        return self._send_goal_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        اگر نہیں goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        
        # Get result future
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(
            f'Received feedback: {feedback_msg.feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    
    # Send goal
    future = action_client.send_goal(10)
    
    # Spin تک goal ہے complete
    rclpy.spin_until_future_complete(action_client, future)
    
    action_client.destroy_node()
    rclpy.shutdown()
```
## تال لالہ

### A ِ NSRWR MAUS C ++
```cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/ایکشن/fibonacci.hpp>

class FibonacciActionServer : public rclcpp::نود
{
public:
    using Fibonacci = example_interfaces::ایکشن::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

    FibonacciActionServer() : نود("fibonacci_action_server")
    {
        using namespace std::placeholders;

        یہ->action_server_ = rclcpp_action::create_server<Fibonacci>(
            یہ->get_node_base_interface(),
            یہ->get_node_clock_interface(),
            یہ->get_node_logging_interface(),
            یہ->get_node_waitables_interface(),
            "fibonacci",
            std::bind,
            std::bind,
            std::bind);
    }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        RCLCPP_INFO, "Received goal request کے ساتھ order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO, "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        using namespace std::placeholders;
        // یہ needs کو return quickly کو avoid blocking کا/کی executor
        std::thread{std::bind, goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO, "Executing goal");

        // Create messages
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto result = std::make_shared<Fibonacci::Result>();

        // Initialize sequence
        feedback->sequence = {0, 1};
        
        auto goal = goal_handle->get_goal();
        
        کے لیے (int میں = 1; میں < goal->order; ++میں) {
            // Check اگر وہاں ہے ایک cancel request
            اگر (goal_handle->is_canceling()) {
                result->sequence = feedback->sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO, "Goal canceled");
                return;
            }

            // Update sequence
            feedback->sequence.push_back(
                feedback->sequence[میں] + feedback->sequence[میں - 1]);
            
            // Publish feedback
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO, "Publishing feedback: %s",
                       std::to_string).c_str());

            // Sleep کو simulate work
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        // Check اگر goal ہے done
        اگر (rclcpp::ok()) {
            result->sequence = feedback->sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO, "Goal succeeded");
        }
    }
};
```
### Aisn ک Laaun ٹ ma یں C ++
```cpp
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/ایکشن/fibonacci.hpp"

class FibonacciActionClient : public rclcpp::نود
{
public:
    using Fibonacci = example_interfaces::ایکشن::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    explicit FibonacciActionClient(const std::string & action_name)
    : نود("fibonacci_action_client")
    {
        یہ->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
            یہ->get_node_base_interface(),
            یہ->get_node_graph_interface(),
            یہ->get_node_logging_interface(),
            یہ->get_node_waitables_interface(),
            action_name);
    }

    void send_goal() {
        using namespace std::placeholders;

        یہ->timer_ = یہ->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind);
        
        اگر)) {
            RCLCPP_ERROR, "ایکشن server نہیں available بعد waiting");
            return;
        }

        // Create goal
        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 10;

        // Set callbacks
        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind;
        send_goal_options.feedback_callback =
            std::bind;
        send_goal_options.result_callback =
            std::bind;
        
        // Send goal
        RCLCPP_INFO, "Sending goal");
        یہ->future_goal_handle_ =
            یہ->client_ptr_->async_send_goal(goal_msg, send_goal_options);
            
        // Cancel timer بعد پہلا execution
        یہ->timer_->cancel();
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr future_goal_handle_;

    void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future) {
        auto goal_handle = future.get();
        اگر (!goal_handle) {
            RCLCPP_ERROR, "Goal تھا rejected کے ذریعے server");
        } else {
            RCLCPP_INFO, "Goal accepted کے ذریعے server, waiting کے لیے result");
        }
    }

    void feedback_callback(
        GoalHandleFibonacci::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
        RCLCPP_INFO, "Received feedback: %s",
                   std::to_string).c_str());
    }

    void result_callback(const GoalHandleFibonacci::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO, "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR, "Goal تھا aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR, "Goal تھا canceled");
                return;
            default:
                RCLCPP_ERROR, "Unknown result code");
                return;
        }

        RCLCPP_INFO, "Result received:");
        کے لیے (auto number : result.result->sequence) {
            RCLCPP_INFO, " %ld", number);
        }
    }
};
```
## اعنیی ی ایم آر آئی ضی کے ماؤبی ق ساسام

### اعنی ی ی مسٹر ضی کے MaUab ق ٹیل دجونا

1.
2. Define آپ کا ایکشن میں ایک `.ایکشن` file:
```
# Navigation.ایکشن
float32 target_x
float32 target_y
float32 target_theta
---
string status
int32 error_code
---
float32 current_x
float32 current_y
float32 current_theta
float32 distance_remaining
```
3. اپ ڈیٹ ڈیٹ ڈیٹ آپ ک a پیکیج. XML:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
4. اپ ڈیٹ ڈیٹ ڈیٹ cmakelists.txt:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "ایکشن/Navigation.ایکشن"
)
```
## پھr
```python
سے my_robot_msgs.ایکشن import Navigation

class NavigationActionServer:
    def __init__(self):
        super().__init__('navigation_action_server')
        self._action_server = ActionServer(
            self,
            Navigation,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        # Validate goal
        اگر goal_request.target_x < 0 یا goal_request.target_y < 0:
            self.get_logger().warn('Invalid navigation goal')
            return GoalResponse.REJECT
        
        self.get_logger().info(f'Accepting navigation goal: {goal_request.target_x}, {goal_request.target_y}')
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing navigation goal...')
        
        feedback_msg = Navigation.Feedback()
        result = Navigation.Result()
        
        # Simulate navigation
        کے لیے step میں range(100):
            اگر goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.status = 'CANCELED'
                result.error_code = 1
                return result
            
            # Update feedback
            feedback_msg.current_x = goal_handle.request.target_x * (step / 100.0)
            feedback_msg.current_y = goal_handle.request.target_y * (step / 100.0)
            feedback_msg.distance_remaining = (1 - step/100.0) * 10.0  # Simplified
            
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate movement
            time.sleep(0.1)
        
        # Success
        goal_handle.succeed()
        result.status = 'SUCCESS'
        result.error_code = 0
        return result
```
## اعل ی ی کے کے کے کے کے یش یش یش یش یش یش پیٹ یش پیٹ پیٹ پیٹ

### AISN کے SAAT ھ JAL جیحی ALUL کی قط قط AR یں
```python
import queue
سے rclpy.ایکشن import ActionServer, GoalResponse
سے example_interfaces.ایکشن import Fibonacci

class PriorityActionServer:
    def __init__(self):
        super().__init__('priority_action_server')
        
        # Priority queue کے لیے goals
        self.goal_queue = queue.PriorityQueue()
        self.current_goal = None
        
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        # کے لیے یہ مثال, higher order = higher priority
        priority = -goal_request.order
        self.goal_queue.put((priority, goal_request))
        
        # Accept تمام goals کو handle priority internally
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # یہ کرے گا implement priority-based goal handling
        pass

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT
```
### AISN کے SAAT ھ MATADAD آ RAA ء کی A ق SAM
```python
سے example_interfaces.ایکشن import Fibonacci

class DetailedFeedbackServer:
    def __init__(self):
        super().__init__('detailed_feedback_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci_detailed',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        self.get_logger().info
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Fibonacci goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        کے لیے میں میں range(1, goal_handle.request.order):
            اگر goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.sequence = feedback_msg.sequence
                return result

            # Update sequence
            next_value = feedback_msg.sequence[میں] + feedback_msg.sequence[میں-1]
            feedback_msg.sequence.append(next_value)
            
            # Publish detailed feedback
            goal_handle.publish_feedback(feedback_msg)
            
            # Log progression
            self.get_logger().info
            
            # Simulate computation time
            time.sleep(0.5)

        # Success
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Fibonacci complete: {result.sequence}')
        
        return result
```
## اِن کی باؤباہ ک تو اسنبالن ے ماؤس غ لی

### lochbw ط AIDN srwr کے SAAATAT غ LI SS ے NMAUN ے کے کے
```python
سے rclpy.ایکشن import ActionServer, GoalResponse, CancelResponse
سے example_interfaces.ایکشن import Fibonacci

class RobustActionServer:
    def __init__(self):
        super().__init__('robust_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci_robust',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        # Validate goal parameters
        اگر goal_request.order < 0:
            self.get_logger().error(f'Invalid order value: {goal_request.order}')
            return GoalResponse.REJECT
        
        اگر goal_request.order > 100:  # Prevent excessive computation
            self.get_logger().warn
        
        self.get_logger().info
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Starting robust execution...')
        
        feedback_msg = Fibonacci.Feedback()
        result = Fibonacci.Result()
        
        try:
            # Initialize sequence
            اگر goal_handle.request.order <= 0:
                result.sequence = []
                goal_handle.succeed()
                return result
            elif goal_handle.request.order == 1:
                result.sequence = [0]
                goal_handle.succeed()
                return result
            else:
                feedback_msg.sequence = [0, 1]

            # Execute کے ساتھ error handling
            کے لیے میں میں range(1, goal_handle.request.order):
                # Check کے لیے cancellation
                اگر goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.sequence = feedback_msg.sequence
                    return result

                # Calculate next Fibonacci number
                next_value = feedback_msg.sequence[میں] + feedback_msg.sequence[میں-1]
                
                # Check کے لیے overflow (simplified)
                اگر next_value < 0:  # Simplified overflow check
                    self.get_logger().error
                    goal_handle.abort()
                    result.sequence = feedback_msg.sequence
                    return result

                feedback_msg.sequence.append(next_value)
                
                # Publish feedback periodically
                اگر میں % 5 == 0:  # Publish every 5 steps کو reduce overhead
                    goal_handle.publish_feedback(feedback_msg)
                
                # Simulate processing time
                time.sleep(0.01)

        except Exception کے طور پر e:
            self.get_logger().error
            goal_handle.abort()
            result.sequence = feedback_msg.sequence
            return result

        # Success
        goal_handle.succeed()
        result.sequence = feedback_msg.sequence
        self.get_logger().info} numbers')
        
        return result
```
## اِن منشرنگ اورسسیس ایس بِکنیی

### ک اِنسان ڈ- لالال in ٹoul کے ll یے a یکش n ز
```bash
# List تمام ایکشنز
ros2 ایکشن list

# Get information about ایک specific ایکشن
ros2 ایکشن info /fibonacci

# Send ایک goal کو ایک ایکشن
ros2 ایکشن send_goal /fibonacci example_interfaces/ایکشن/Fibonacci "{order: 5}"

# Show ایکشن type definition
ros2 interface show example_interfaces/ایکشن/Fibonacci
```
### یٹ n ق sm کی maylwamat
```bash
# View کا/کی structure کا ایک ایکشن
ros2 interface show example_interfaces/ایکشن/Fibonacci
```
## ک اراپراڈ گی کے تالٹ

###
```python
import time
سے rclpy.ایکشن import ActionServer
سے example_interfaces.ایکشن import Fibonacci

class EfficientActionServer:
    def __init__(self):
        super().__init__('efficient_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci_efficient',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        # Quick validation without heavy computation
        اگر goal_request.order < 0 یا goal_request.order > 1000:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        start_time = time.time()
        
        feedback_msg = Fibonacci.Feedback()
        result = Fibonacci.Result()
        
        # Optimize feedback frequency based پر task length
        feedback_interval = max(1, goal_handle.request.order // 20)  # Max 20 feedbacks
        
        try:
            # Pre-allocate list اگر possible
            اگر goal_handle.request.order > 0:
                sequence = [0] * goal_handle.request.order
                اگر goal_handle.request.order > 1:
                    sequence[1] = 1
                
                کے لیے میں میں range(2, goal_handle.request.order):
                    اگر goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.sequence = sequence[:میں]
                        return result

                    sequence[میں] = sequence[میں-1] + sequence[میں-2]
                    
                    # Send feedback پر intervals کو reduce overhead
                    اگر میں % feedback_interval == 0:
                        feedback_msg.sequence = sequence[:میں+1]
                        goal_handle.publish_feedback(feedback_msg)
            else:
                sequence = []

        except Exception کے طور پر e:
            self.get_logger().error(f'Execution failed: {e}')
            goal_handle.abort()
            result.sequence = sequence
            return result

        # Success
        goal_handle.succeed()
        result.sequence = sequence
        end_time = time.time()
        
        self.get_logger().info(
            f'Completed Fibonacci({len(result.sequence)}) میں {end_time-start_time:.2f}s')
        
        return result
```
## baa ہ ٹra ی n ط ri ک ar کے ll یے iaun ز

1.
2
3.
4.
5
6
7.

## خ LAA صہ

یہ الل l ی کے کے sat ھ ھ ، ، ، ، r بچلری ی n سلر ، am aml ہیں۔ as آئیڈی ز کے ِ چ چ چ چ چ چ چ چ کے کے کے کے کے کے کے ہ ہ ہ ذی ذی ذی ذی ذی ذی ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ذی ہ ذی ذی ذی ذی ذی ہ ہ ذی ہ ہ چ ذی ذی ذی ذی ذی ذی ذی ذی کے کے کے کے چ کے کے کے ذی ذی ذی کے کے کے کے کے
