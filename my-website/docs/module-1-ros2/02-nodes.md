# ROS 2: Nodes

## Understanding ROS 2 Nodes

In ROS 2, a node is an executable that performs a specific task. Nodes are fundamental components of any ROS 2 system, designed to be modular and reusable.

### Key Characteristics of Nodes:
- **Modularity:** Each node typically handles a single, well-defined function (e.g., a camera driver, a motor controller, a navigation algorithm).
- **Communication:** Nodes communicate with each other using various mechanisms such as topics, services, and actions.
- **Execution:** Nodes run independently and can be started, stopped, and managed using ROS 2 tools.

### Creating a Simple ROS 2 Node (Python Example):

Here's a basic example of a ROS 2 node written in Python using `rclpy`.

```python
import rclpy
from rclpy.node import Node

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    my_publisher = MyPublisher()

    rclpy.spin(my_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Explanation of the Code:
- `import rclpy` and `from rclpy.node import Node`: Imports necessary ROS 2 Python client library modules.
- `class MyPublisher(Node):`: Defines a new node class that inherits from `rclpy.node.Node`.
- `super().__init__('my_publisher')`: Initializes the node with the name 'my_publisher'.
- `self.create_publisher(String, 'topic', 10)`: Creates a publisher that sends messages of type `String` to a topic named 'topic' with a queue size of 10.
- `self.create_timer(timer_period, self.timer_callback)`: Sets up a timer to call `timer_callback` every 0.5 seconds.
- `timer_callback()`: The function that gets called by the timer, constructs a message, publishes it, and logs information.
- `rclpy.init(args=args)` and `rclpy.spin(my_publisher)`: Initializes the ROS 2 Python client library and keeps the node alive until it's explicitly shut down or `Ctrl+C` is pressed.

### Further Reading:
- [ROS 2 Nodes (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [ROS 2 Nodes (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Writing-A-Simple-Python-Publisher-And-Subscriber.html)
