# ROS 2: Topics

## Introduction to ROS 2 Topics

Topics are the most common way for nodes to asynchronously exchange messages in ROS 2. They operate on a publish/subscribe model, where nodes publish messages to a named topic, and other nodes subscribe to that topic to receive the messages.

### Key Concepts:
- **Publisher:** A node that sends messages to a topic.
- **Subscriber:** A node that receives messages from a topic.
- **Message Type:** The data structure of the messages being exchanged (e.g., `std_msgs/String`, `geometry_msgs/Twist`). ROS 2 uses `.msg` files to define custom message types.
- **Topic Name:** A unique identifier for the communication channel.

### How Topics Work:
1. A publisher node creates a message and publishes it to a specific topic.
2. Any subscriber nodes that are subscribed to that same topic will receive the message.
3. The communication is one-to-many, meaning a single publisher can send messages to multiple subscribers, and a single subscriber can receive messages from multiple publishers.

### Creating a Simple ROS 2 Publisher and Subscriber (Python Example):

#### Publisher Node:

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
        msg.data = 'Hello, world! %d' % self.i
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

#### Subscriber Node:

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

### Further Reading:
- [ROS 2 Topics (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [ROS 2 Topics (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Writing-A-Simple-Python-Publisher-And-Subscriber.html)
