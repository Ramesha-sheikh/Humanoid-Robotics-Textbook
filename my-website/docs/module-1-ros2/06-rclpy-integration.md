# ROS 2: rclpy Integration

## Introduction to rclpy

`rclpy` is the Python client library for ROS 2, providing a Pythonic interface to interact with the ROS 2 ecosystem. It allows developers to write ROS 2 nodes, publishers, subscribers, services, and actions using Python, leveraging its ease of use and extensive libraries.

### Key Features of rclpy:
- **Pythonic Interface:** Designed to be intuitive for Python developers.
- **Integration with ROS 2 Core:** Built on top of `rcl` (ROS Client Library) for seamless integration with the underlying C++ ROS 2 implementation.
- **Asynchronous Programming:** Supports asynchronous operations, which is crucial for efficient robotic applications.
- **Extensibility:** Allows for the creation of complex robotic behaviors and applications using Python's rich ecosystem.

### Core rclpy Concepts:
- **`rclpy.init()` and `rclpy.shutdown()`:** Functions to initialize and deinitialize the ROS 2 Python client library.
- **`rclpy.node.Node`:** The base class for creating ROS 2 nodes in Python.
- **`create_publisher()` and `create_subscription()`:** Methods of the `Node` class to create publishers and subscribers.
- **`create_service()` and `create_client()`:** Methods for creating service servers and clients.
- **`rclpy.spin()` and `rclpy.spin_once()`:** Functions to process ROS 2 callbacks (e.g., messages, service requests).

### Example: Combining Concepts with rclpy

Let's look at an example that combines a publisher, subscriber, and a service client within a single Python node, demonstrating `rclpy` integration.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts

class CombinedNode(Node):
    def __init__(self):
        super().__init__('combined_node')

        # Publisher
        self.publisher_ = self.create_publisher(String, 'chat_topic', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publisher_callback)
        self.i = 0

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'chat_topic',
            self.subscriber_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Service Client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('add_two_ints service not available, waiting again...')

    def publisher_callback(self):
        msg = String()
        msg.data = f'Hello from CombinedNode! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}'')
        self.i += 1

        # Call service every 5 messages
        if self.i % 5 == 0:
            self.send_service_request(self.i, self.i + 1)

    def subscriber_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}'')

    def send_service_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.cli.call_async(request)
        # It's generally not recommended to block in callbacks, but for demonstration...
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info(
                f'Service Result: {request.a} + {request.b} = {self.future.result().sum}'
            )
        else:
            self.get_logger().error('Service call failed %r' % (self.future.exception(),))

def main(args=None):
    rclpy.init(args=args)
    node = CombinedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Further Reading:
- [ROS 2 Python Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Programming-Language-Python/index.html)
- [rclpy Documentation](https://docs.ros.org/en/humble/p/rclpy/)
