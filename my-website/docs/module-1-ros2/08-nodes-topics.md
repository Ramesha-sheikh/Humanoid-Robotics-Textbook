# ROS 2: Nodes & Topics

## Nodes and Topics: The Foundation of ROS 2 Communication

In ROS 2, **nodes** are individual executable processes that perform specific tasks, while **topics** are the primary mechanism for nodes to asynchronously exchange messages. Together, they form the core of how a distributed robotic system communicates and operates.

### ROS 2 Nodes: Modular Computation Units

Think of a ROS 2 node as a single program that performs a particular function, like a camera driver, a motor controller, or a path planning algorithm. This modular design has several benefits:

-   **Reusability:** Individual nodes can be reused in different robotic applications.
-   **Fault Isolation:** If one node crashes, it doesn't necessarily bring down the entire system.
-   **Distributed Processing:** Nodes can run on different machines or even different robots, communicating over a network.

**Creating a Node (Python - `rclpy`):**

```python
import rclpy
from rclpy.node import Node

# Define a simple node class
class MyNode(Node):
    def __init__(self):
        super().__init__('my_python_node') # Initialize the node with a name
        self.get_logger().info('My Python Node has started!')

def main(args=None):
    rclpy.init(args=args) # Initialize the ROS 2 client library
    node = MyNode()
    rclpy.spin(node) # Keep the node alive until shutdown
    node.destroy_node()
    rclpy.shutdown() # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()
```

### ROS 2 Topics: Asynchronous Data Streams

Topics enable nodes to send and receive information without direct knowledge of each other. This is achieved through a publish-subscribe pattern:

-   **Publisher:** A node that sends messages to a topic.
-   **Subscriber:** A node that receives messages from a topic.
-   **Message Type:** The format of data exchanged over a topic (e.g., `String`, `Twist`, `Image`). These are defined in `.msg` files.

#### How it Works:
1.  A publisher node creates a message (e.g., a string, a velocity command, an image).
2.  It then publishes this message to a named topic (e.g., `/chatter`, `/cmd_vel`, `/camera/image_raw`).
3.  Any node that is subscribed to that exact topic will receive the message.

This communication is one-to-many, meaning one publisher can send data to many subscribers, and one subscriber can receive data from many publishers on the same topic.

**Creating a Publisher (Python - `rclpy`):**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the standard String message type

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        # Create a publisher to the topic 'my_topic' with String messages
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String() # Create a new String message
        msg.data = f'Hello ROS 2 from Python: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}'')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Creating a Subscriber (Python - `rclpy`):**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        # Create a subscriber to the topic 'my_topic' with String messages
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10) # QoS history depth
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}'')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Roman Urdu Explanation:

`Nodes woh chote programs hote hain jo robot mein alag-alag kaam karte hain, jaise camera se data lena ya motor ko chalana. Topics ek rasta hota hai jahan yeh nodes ek doosre ko messages bhejte hain, jaise ek news channel jahan log khabar bhejte hain aur doosre sunte hain. Messages mein data hota hai jo nodes share karte hain.`

### Multiple Choice Questions (MCQs):

1.  **What is the primary function of a ROS 2 Node?**
    a) To manage all ROS 2 communication centrally.
    b) To perform a single, specific computational task.
    c) To define the data types for messages.
    d) To provide a graphical user interface for ROS 2.
    *Correct Answer: b) To perform a single, specific computational task.*

2.  **Which communication pattern do ROS 2 Topics primarily use?**
    a) Request/Response
    b) Client/Server
    c) Publish/Subscribe
    d) Peer-to-Peer
    *Correct Answer: c) Publish/Subscribe*

3.  **If Node A sends data to Node B via a topic, Node A is acting as a:**
    a) Subscriber
    b) Client
    c) Server
    d) Publisher
    *Correct Answer: d) Publisher*

4.  **What kind of communication is suitable for data streams that don't require an immediate reply?**
    a) Services
    b) Actions
    c) Topics
    d) Parameters
    *Correct Answer: c) Topics*

5.  **Which Python library is used to create ROS 2 nodes and interact with topics?**
    a) `rospy`
    b) `rclcpp`
    c) `rclpy`
    d) `std_msgs`
    *Correct Answer: c) `rclpy`*

### Further Reading:
- [ROS 2 Nodes (Conceptual)](https://docs.ros.org/en/humble/Concepts/Basic-Concepts/About-Nodes.html)
- [ROS 2 Topics (Conceptual)](https://docs.ros.org/en/humble/Concepts/Basic-Concepts/About-Topics.html)
