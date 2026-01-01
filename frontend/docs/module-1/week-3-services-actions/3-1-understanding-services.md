---
sidebar_position: 1
difficulty: beginner
---

# 3.1: Understanding ROS 2 Services

## Overview

This submodule covers ROS 2 services, which provide a request/reply communication pattern different from the asynchronous publish/subscribe model of topics. Services are synchronous and ideal for operations that require direct responses.

## Learning Objectives

By the end of this submodule, you will:
- Understand the service communication pattern in ROS 2
- Create and implement services in both Python and C++
- Work with built-in and custom service types
- Compare service usage with topics and actions
- Apply appropriate QoS policies for services

## Service Communication Pattern

ROS 2 services implement a **request/reply** communication pattern, which is fundamentally different from topics:

- **Synchronous**: The client waits for a response from the server
- **Request/Reply**: Client sends a request, server processes and returns a reply
- **One-to-One**: Each request is handled by a single server
- **Blocking**: Client execution pauses until response is received

### When to Use Services vs Topics

| Use Services When | Use Topics When |
|-------------------|-----------------|
| Need guaranteed response | One-to-many communication |
| Request-specific data | Continuous data stream |
| Synchronous operations | Asynchronous operations |
| Configuration changes | State updates |
| Validation or processing | Sensor data |

## Creating Services in Python

### Basic Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback)
        self.get_logger().info('Service server started')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()
```

### Basic Service Client

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    
    # Create request and call service
    future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    
    # Wait for response
    rclpy.spin_until_future_complete(minimal_client, future)
    
    response = future.result()
    minimal_client.get_logger().info(
        f'Result of {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
    
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Services in C++

### Service Server in C++

```cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class MinimalService : public rclcpp::Node
{
public:
    MinimalService() : Node("minimal_service")
    {
        service_ = create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            [this](const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                   example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {
                response->sum = request->a + request->b;
                RCLCPP_INFO(this->get_logger(), 
                           "Incoming request: %ld + %ld = %ld", 
                           request->a, request->b, response->sum);
            });
        RCLCPP_INFO(this->get_logger(), "Service server started");
    }

private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};
```

### Service Client in C++

```cpp
#include <chrono>
#include <cinttypes>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

class MinimalClient : public rclcpp::Node
{
public:
    MinimalClient() : Node("minimal_client")
    {
        client_ = create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        
        // Wait for service to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        
        // Create request and send it
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = 2;
        request->b = 3;
        
        future_ = client_->async_send_request(request);
    }

    void execute() {
        // Wait for response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_) == 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), 
                       "Result: %" PRId64, future_.get()->sum);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future_;
};
```

## Custom Service Types

### Creating Custom Services

1. Create a srv directory in your package
2. Define your service in a `.srv` file:
```
# AddThreeInts.srv
int64 a
int64 b
int64 c
---
int64 sum
```

3. Update your package.xml:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

4. Update CMakeLists.txt:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddThreeInts.srv"
)
```

### Using Custom Services

```python
from my_robot_msgs.srv import AddThreeInts

# Service server with custom service
class CustomServiceServer(Node):
    def __init__(self):
        super().__init__('custom_service_server')
        self.srv = self.create_service(
            AddThreeInts, 
            'add_three_ints', 
            self.add_three_ints_callback)

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c
        self.get_logger().info(
            f'Adding: {request.a} + {request.b} + {request.c} = {response.sum}')
        return response

# Service client with custom service
class CustomServiceClient(Node):
    def __init__(self):
        super().__init__('custom_service_client')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        self.req = AddThreeInts.Request()

    def send_request(self, a, b, c):
        self.req.a = a
        self.req.b = b
        self.req.c = c
        return self.cli.call_async(self.req)
```

## Service Quality of Service (QoS)

Services have their own QoS settings distinct from topics:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Create QoS for services
service_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)

# Apply to service creation
self.srv = self.create_service(
    AddTwoInts,
    'add_two_ints',
    self.add_two_ints_callback,
    qos_profile=service_qos
)
```

### Important Service QoS Considerations

- **Reliability**: Usually RELIABLE for services to ensure requests/replies aren't lost
- **History**: KEEP_LAST with small depth is typical since old requests are irrelevant
- **Depth**: Controls how many service calls can be queued

## Error Handling in Services

### Service Server with Error Handling

```python
from example_interfaces.srv import AddTwoInts

class RobustServiceServer(Node):
    def __init__(self):
        super().__init__('robust_service_server')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        try:
            # Validate inputs
            if not isinstance(request.a, int) or not isinstance(request.b, int):
                self.get_logger().error('Invalid input types')
                response.sum = 0
                return response
            
            # Perform operation
            result = request.a + request.b
            
            # Check for overflow (simplified)
            if result > 2**63 - 1 or result < -2**63:
                self.get_logger().error('Integer overflow detected')
                response.sum = 0
                return response
            
            response.sum = result
            self.get_logger().info(f'Result: {request.a} + {request.b} = {response.sum}')
            
        except Exception as e:
            self.get_logger().error(f'Error in service callback: {e}')
            response.sum = 0  # Return default value on error
        
        return response
```

### Service Client with Error Handling

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from rclpy.executors import ExternalShutdownException

class RobustServiceClient(Node):
    def __init__(self):
        super().__init__('robust_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        self.timeout = 5.0  # seconds

    def call_service_with_timeout(self, a, b, timeout=None):
        if timeout is None:
            timeout = self.timeout
            
        if not self.cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('Service not available')
            return None
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        # Call service asynchronously
        future = self.cli.call_async(request)
        
        # Wait with timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        try:
            response = future.result()
            return response
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    client = RobustServiceClient()
    
    # Call service
    result = client.call_service_with_timeout(10, 20)
    if result:
        print(f"Service result: {result.sum}")
    else:
        print("Service call failed or timed out")
    
    client.destroy_node()
    rclpy.shutdown()
```

## Advanced Service Patterns

### Service with Multiple Clients

```python
import threading
import time
from example_interfaces.srv import AddTwoInts

class MultiClientService(Node):
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
        with self.lock:
            self.request_counter += 1
            request_id = self.request_counter
        
        self.get_logger().info(
            f'Request #{request_id}: {request.a} + {request.b}')
        
        # Simulate processing time
        time.sleep(0.1)
        
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Response #{request_id}: {response.sum}')
        
        return response
```

## Service Monitoring and Debugging

### Command-Line Tools for Services

```bash
# List all services
ros2 service list

# Get information about a specific service
ros2 service info /add_two_ints

# Call a service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# Show service type
ros2 service type /add_two_ints

# Find nodes providing a service
ros2 service nodes /add_two_ints
```

### Service Type Information

```bash
# Get detailed service definition
ros2 interface show example_interfaces/srv/AddTwoInts
```

## Best Practices for Services

1. **Appropriate Use**: Use services for operations that require immediate responses
2. **Timeout Handling**: Always implement timeout logic in clients
3. **Error Handling**: Properly handle exceptions in service callbacks
4. **Performance**: Keep service operations lightweight to avoid blocking
5. **Validation**: Validate service request parameters
6. **Documentation**: Document service interfaces clearly

## Summary

This submodule covered ROS 2 services, their implementation in Python and C++, custom service types, QoS considerations, and best practices. Services are ideal for request/reply communication scenarios where synchronous responses are needed. In the next submodule, we'll explore ROS 2 actions, which combine aspects of both topics and services for goal-oriented tasks.