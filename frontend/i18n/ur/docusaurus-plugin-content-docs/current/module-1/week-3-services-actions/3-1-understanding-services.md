---


sidebar_position: 1
difficulty: beginner


---
# 3.1: روس 2 سروس کو سوسمنا

## ج a ج

یہ ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج کی کی کی کی کی کی کی کی کی کی نیچے ج ج maa ڈ l ک a ٹ aauss۔ سروسس ہ ہ ہ ہ m wausaat ssaa ز oavavr آئیڈی کے ہی آپ آپ آپ آپ آپ آپ آپ آپ آپ آپ آپ آپ آپ ز آپ آپ rauni ز ک sraus jrasat readaiml کی ض ض ض ض ض ض ض ض کی کی کی کی کی

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- عمعو کہ/کی سروس موسلٹ اولرن ما ما
- OWR -ONA فذ ک RA یں SROSS ز MAH DONWAU ں R OWR OR C ++
- کam کے ssaٹ ھ bulٹ- maus آvrss ک sausm srsrsos کی
- Compare سروس usage کے ساتھ ٹاپکس اور ایکشنز
- مناسب QoS پالیسیاں لگائیں کے Lیے Srrosز

## سروس موولات بعدق

روس 2 سروسس ک ک ک ک ک ک ک ک ک ک ک //////////////////////////// ج ج ج/ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج ج

- ** ہ m آہ n گی **: ک a/کی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک کی کی کی کی کی کی کی کی کی کی کی کی کی ک کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی ک ک کی کی کی کی ک ک کی ک کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی نیچے کی کی کی کی کی کی کی کی کی کی.
۔
-**-ک so-ia یک **: ir ad dd dd dd a
- **Blocking**: Client execution pauses تک response ہے received

### ک B ک V Vasatamail ک ri یں srsrosss bumaabli ٹ aa پک s

| سروسس ک bی | یسعمال ک یں پک s ک Be |
| ----------------------- | ----------------------- |
| گ ارن ٹیڈ ج wab کی ض ض ض ض ض کی کی کی کی ہے ia یک- uso- b ہ t ss ے mwaaulat |
| دلدوست ساؤت متالق | | Mastی ڈیٹ a ڈیٹ a asaurium |
| ہم وِت ساسا آپ آپ ri یش n | غی r متال ز آپ آپ آپ r یش n ز |
| تال تبدقالی | raausast کی کی کی ک ar ی |
| توو ی a arros ی sn گ | سنسر ڈیٹ اے |

## سروسس کو تال دِس رِسسا ہے

### بن ی AAD ی سروس سرور
```python
import rclpy
سے rclpy.نود import نود
سے example_interfaces.srv import AddTwoInts

class MinimalService:
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback)
        self.get_logger().info

    def add_two_ints_callback(self, request, response):
        response.sum = request.ایک + request.b
        self.get_logger().info
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()
```
### bin ی aad ی srrws ک laiun ٹ
```python
import sys
import rclpy
سے rclpy.نود import نود
سے example_interfaces.srv import AddTwoInts

class MinimalClient:
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Wait کے لیے سروس کو ہونا available
        جب تک نہیں self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info
        
        self.req = AddTwoInts.Request()

    def send_request:
        self.req.ایک = ایک
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    
    # Create request اور call سروس
    future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    
    # Wait کے لیے response
    rclpy.spin_until_future_complete(minimal_client, future)
    
    response = future.result()
    minimal_client.get_logger().info(
        f'Result کا {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
    
    minimal_client.destroy_node()
    rclpy.shutdown()

اگر __name__ == '__main__':
    main()
```
## سروسس ز ماؤس ایس ایس ی ++ بونانا

### سروس سرور مِس سیس ++
```cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class MinimalService : public rclcpp::نود
{
public:
    MinimalService() : نود("minimal_service")
    {
        service_ = create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            [یہ](const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                   example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {
                response->sum = request->ایک + request->b;
                RCLCPP_INFO, 
                           "Incoming request: %ld + %ld = %ld", 
                           request->ایک, request->b, response->sum);
            });
        RCLCPP_INFO, "سروس server started");
    }

private:
    rclcpp::سروس<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};
```
### -srros ک Laaun ٹ MAH C ++
```cpp
#include <chrono>
#include <cinttypes>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

class MinimalClient : public rclcpp::نود
{
public:
    MinimalClient() : نود("minimal_client")
    {
        client_ = create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        
        // Wait کے لیے سروس کو ہونا available
        جب تک (!client_->wait_for_service(1s)) {
            اگر (!rclcpp::ok()) {
                RCLCPP_ERROR, "Interrupted جب تک waiting کے لیے سروس");
                return;
            }
            RCLCPP_INFO, "سروس نہیں available, waiting again...");
        }
        
        // Create request اور send یہ
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->ایک = 2;
        request->b = 3;
        
        future_ = client_->async_send_request(request);
    }

    void execute() {
        // Wait کے لیے response
        اگر, future_) == 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO, 
                       "Result: %" PRId64, future_.get()->sum);
        } else {
            RCLCPP_ERROR, "Failed کو call سروس");
        }
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future_;
};
```
## ک Saum Scrsos کی کی Sasam

### اعنی ی ی مسٹر ضی کے mauab ق srsrosss Bunana

1. ia یک srv ڈ aa ئ ra کٹ ra کٹ bnaa ئیں maus آپ ک
2. Define آپ کا سروس میں ایک `.srv` file:
```
# AddThreeInts.srv
int64 ایک
int64 b
int64 c
---
int64 sum
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
  "srv/AddThreeInts.srv"
)
```
### ک ساسم ایس ایس آر ایس ایس ک a ک asatamail arta ے ہ vi
```python
سے my_robot_msgs.srv import AddThreeInts

# سروس server کے ساتھ custom سروس
class CustomServiceServer:
    def __init__(self):
        super().__init__('custom_service_server')
        self.srv = self.create_service(
            AddThreeInts, 
            'add_three_ints', 
            self.add_three_ints_callback)

    def add_three_ints_callback(self, request, response):
        response.sum = request.ایک + request.b + request.c
        self.get_logger().info(
            f'Adding: {request.ایک} + {request.b} + {request.c} = {response.sum}')
        return response

# سروس client کے ساتھ custom سروس
class CustomServiceClient:
    def __init__(self):
        super().__init__('custom_service_client')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')
        
        جب تک نہیں self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info
        
        self.req = AddThreeInts.Request()

    def send_request:
        self.req.ایک = ایک
        self.req.b = b
        self.req.c = c
        return self.cli.call_async(self.req)
```
## سروس ک ایک مایاور

سروسس رِس ے ے ہیں ہیں ہیں ہیں ہیں qos کی کی t t t t t t t t t t کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی
```python
سے rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Create QoS کے لیے سروسز
service_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)

# Apply کو سروس creation
self.srv = self.create_service(
    AddTwoInts,
    'add_two_ints',
    self.add_two_ints_callback,
    qos_profile=service_qos
)
```
### اعم سروس یسوست

۔
.
.

## غ l طی ک l طی ک v snanbauln ے ma یں غ l طی

### -srws srwr کے SAAT غ غ l طی غ l طی ss ے nmaun ے کے
```python
سے example_interfaces.srv import AddTwoInts

class RobustServiceServer:
    def __init__(self):
        super().__init__('robust_service_server')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        try:
            # Validate inputs
            اگر نہیں isinstance یا نہیں isinstance(request.b, int):
                self.get_logger().error('Invalid input types')
                response.sum = 0
                return response
            
            # Perform operation
            result = request.ایک + request.b
            
            # Check کے لیے overflow (simplified)
            اگر result > 2**63 - 1 یا result < -2**63:
                self.get_logger().error('Integer overflow detected')
                response.sum = 0
                return response
            
            response.sum = result
            self.get_logger().info
            
        except Exception کے طور پر e:
            self.get_logger().error
            response.sum = 0  # Return default value پر error
        
        return response
```
### -srws ک laaaiun ٹ کے SAAT ھ غ غ l طی ss ے nmaun ے کے
```python
import rclpy
سے rclpy.نود import نود
سے example_interfaces.srv import AddTwoInts
سے rclpy.executors import ExternalShutdownException

class RobustServiceClient:
    def __init__(self):
        super().__init__('robust_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        self.timeout = 5.0  # seconds

    def call_service_with_timeout:
        اگر timeout ہے None:
            timeout = self.timeout
            
        اگر نہیں self.cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error
            return None
        
        request = AddTwoInts.Request()
        request.ایک = ایک
        request.b = b
        
        # Call سروس asynchronously
        future = self.cli.call_async(request)
        
        # Wait کے ساتھ timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        try:
            response = future.result()
            return response
        except Exception کے طور پر e:
            self.get_logger().error
            return None

def main(args=None):
    rclpy.init(args=args)
    client = RobustServiceClient()
    
    # Call سروس
    result = client.call_service_with_timeout(10, 20)
    اگر result:
    else:
    
    client.destroy_node()
    rclpy.shutdown()
```
## اعل ی کی کی کے کے کے کے کے کے ن J Kn N J J N N N N N N N N N N N N N N N N N N N N N N N N N N N N ن کی کی کی کی کی کی

### -srws کے saa ؤ t ss ے زی ad ہ ک ک laa ئ n ٹ
```python
import threading
import time
سے example_interfaces.srv import AddTwoInts

class MultiClientService:
    def __init__(self):
        super().__init__('multi_client_service')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback)
        
        # Track client requests
        self.request_counter = 0
        self.lock = threading.Lock()

    def add_two_ints_callback(self, request, response):
        کے ساتھ self.lock:
            self.request_counter += 1
            request_id = self.request_counter
        
        self.get_logger().info(
            f'Request #{request_id}: {request.ایک} + {request.b}')
        
        # Simulate processing time
        time.sleep(0.1)
        
        response.sum = request.ایک + request.b
        self.get_logger().info(
            f'Response #{request_id}: {response.sum}')
        
        return response
```
## سروس ماناآرنگ اواورس ڈیبگنگ

### ک maaunaai ڈ- لال in ٹ oli کے lliss srrosss
```bash
# List تمام سروسز
ros2 سروس list

# Get information about ایک specific سروس
ros2 سروس info /add_two_ints

# Call ایک سروس سے کمانڈ line
ros2 سروس call /add_two_ints example_interfaces/srv/AddTwoInts "{ایک: 1, b: 2}"

# Show سروس type
ros2 سروس type /add_two_ints

# Find نوڈز providing ایک سروس
ros2 سروس نوڈز /add_two_ints
```
### -srws ق sm کی maylwamat
```bash
# Get detailed سروس definition
ros2 interface show example_interfaces/srv/AddTwoInts
```
## بِبیبرن اعاریقہ ک آ آ کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے

1.
2.
3
4.
5.
6.

## خ LAA صہ

یہ الل l ی ں ہیں ہیں کے کے کے کے کے کے کے کے یے یے یے یے یے یے یے یے یے یے یے یے یے یے یے یے جoab جoab جoab جwab جoab جwab جoab جoab id ج جoab idیں جoab جیبد دِدد دِدد دِد دِد دِد دِد دِد دِد دِد دِد دِد دِد دِد دِد دِد دِد دِد دِد دِد دِد دِد دِد دِد دِکوبی بعوبی بعوبی بعوبی بعوبی بعب اِدو بی دید دِکوبی بعوبی بعوبی بعب اِدو بوبی بعوبی بِد دِد دِدو بی دید دِک ہیں ہیں کہ کہ کہ کہ کہ ہیں ہیں ہیں ہیں کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ کہ ہیں ، کہ کہ کے کے کے کے کے کے کے کے پہ J پہ پہ پہ پہ ک ک J ک ک ک ک ک گے گے گے گے ج S ج S ج S ج S ج S ج S ج S ج S ج S ج S ج S ج S ج Junwau ک V Mlaid
