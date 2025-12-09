## ROS 2 Topics: Real-time Data Streaming

💬 **Theory Insight: The Publish-Subscribe Communication Model**

ROS 2 *topics* are the most common way for nodes to exchange real-time data in a publish-subscribe communication model. Imagine a bulletin board where nodes can post information (publish) and other nodes can read that information (subscribe). This asynchronous, many-to-many communication pattern is ideal for streaming continuous data, such as sensor readings (camera images, LiDAR scans, IMU data), joint states, and robot odometry.

Here's how it works:

-   **Publisher**: A node that creates and sends messages on a specific topic. For example, a camera driver node publishes image data to an `/image_raw` topic.
-   **Subscriber**: A node that receives messages from a specific topic. For example, a computer vision node subscribes to `/image_raw` to process images.
-   **Topic Name**: A unique identifier (e.g., `/scan`, `/joint_states`, `/cmd_vel`) that defines the data channel. Nodes publish to and subscribe from specific topic names.
-   **Message Type**: The data structure of the messages being exchanged (e.g., `sensor_msgs/msg/LaserScan` for LiDAR, `geometry_msgs/msg/Twist` for velocity commands). All publishers and subscribers on a given topic must use the same message type.

Topics enable loose coupling between nodes: publishers don't need to know which subscribers are listening, and subscribers don't need to know which publishers are sending. This enhances modularity and fault tolerance in complex robotic systems.

### ROS 2 Topic Communication

```mermaid
graph TD
    A[Sensor Node]
    B[Control Node]
    C[Navigation Node]
    D[Visualization Node]

    A -- Publishes sensor_msgs/LaserScan --> Topic1[/scan]
    A -- Publishes sensor_msgs/Image --> Topic2[/camera/image]

    Topic1 --> C
    Topic2 --> C
    Topic1 --> D

    C -- Publishes geometry_msgs/Twist --> Topic3[/cmd_vel]
    Topic3 --> B
```

### Key Topic Properties

| Property        | Description                                                               | Importance                                                    |
|-----------------|---------------------------------------------------------------------------|---------------------------------------------------------------|
| **Quality of Service (QoS)** | Set of policies governing message delivery (reliability, durability, history) | Crucial for real-time performance, data integrity, and specific application needs |
| **Message Rate**| Frequency at which messages are published                                 | Impacts control loop stability, data freshness, network load  |
| **Latency**     | Time delay between message publication and reception                      | Critical for reactive robot behaviors and teleoperation       |

🎓 **Key Insight: The Critical Role of Quality of Service (QoS) Settings**

While the publish-subscribe model of ROS 2 topics provides great flexibility, the *Quality of Service (QoS) settings* are paramount for ensuring that data is delivered reliably and efficiently according to the specific needs of each communication link. QoS policies allow developers to define critical aspects of message delivery, such as:

-   **Reliability**: Whether messages are guaranteed to arrive (reliable) or if some can be lost for timeliness (best effort). For sensor data streams, best effort might be acceptable, but for critical control commands, reliable is essential.
-   **Durability**: Whether late-joining subscribers receive historical messages (transient local) or only new messages (volatile).
-   **History**: How many messages are kept in the queue (keep last) or if all messages are kept (keep all) until processed.
-   **Liveliness**: How the system detects if a publisher or subscriber is still active.

Incorrect QoS settings can lead to missed commands, stale sensor data, or excessive network overhead. For humanoid robots, particularly in safety-critical applications like human-robot collaboration or autonomous navigation, precisely configuring QoS is not just an optimization but a fundamental requirement for predictable and safe operation. For example, a joint command topic might require reliable delivery, while a high-frequency camera stream might prioritize best-effort for lower latency.

### Code Example: Simple ROS 2 Python Node (Subscriber)

This Python code defines a basic ROS 2 node that subscribes to the `chatter` topic and prints received messages. This complements the `SimplePublisher` node from the previous lesson.

```python
# File: ros2_ws/src/simple_subscriber_pkg/simple_subscriber_pkg/subscriber_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber_node') # Initialize the node with a name
        # Create a subscriber to the 'chatter' topic with a callback function
        self.subscription = self.create_subscription(
            String, # Message type
            'chatter', # Topic name
            self.listener_callback, # Callback function
            10) # QoS profile depth
        self.subscription # prevent unused variable warning
        self.get_logger().info(f"Simple Subscriber Node started, listening to 'chatter' topic.")

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"') # Log the received message

def main(args=None):
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()

    rclpy.spin(simple_subscriber)

    # Destroy the node explicitly
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this code:
1.  Save it as `ros2_ws/src/simple_subscriber_pkg/simple_subscriber_pkg/subscriber_node.py`.
2.  Build your ROS 2 workspace (after setting up `colcon_build`).
3.  Source your ROS 2 environment.
4.  Run the publisher node in one terminal: `ros2 run simple_publisher_pkg publisher_node`
5.  Run this subscriber node in another terminal: `ros2 run simple_subscriber_pkg subscriber_node`

You should see messages from the publisher being received and printed by the subscriber.

🤝 **Practice: Observing ROS 2 Topic Data via FastAPI**

To enable monitoring and debugging of our ROS 2 system through a web interface, our FastAPI backend should be able to expose real-time topic data. This practice exercise involves conceptually extending the FastAPI backend (`backend/main.py`) to simulate subscribing to a ROS 2 topic and returning its latest message. This is a stepping stone towards building a full ROS 2 web interface.

### `curl` Example: Getting Latest Topic Message

Assume our FastAPI backend has an endpoint `/ros2/topic/latest` that takes a `topic_name` as a query parameter and returns the latest simulated message from that topic.

```bash
# Placeholder curl command - replace with actual FastAPI endpoint once ready
# Ensure your FastAPI backend (backend/main.py) is running (e.g., uvicorn main:app --reload)

curl -X GET \\
  "http://127.0.0.1:8000/ros2/topic/latest?topic_name=/chatter" \\
  -H "Content-Type: application/json"
```

**Expected (Simulated) FastAPI Response:**

```json
{
  "status": "success",
  "topic_name": "/chatter",
  "message_type": "std_msgs/String",
  "latest_message": "Hello ROS 2 from Python: 42",
  "timestamp": "2025-12-07T14:45:00Z"
}
```

This `curl` command simulates querying our backend for real-time ROS 2 data. In a full implementation, the FastAPI endpoint would create a temporary ROS 2 subscriber (or utilize a dedicated ROS 2 bridge node) to listen for messages on the specified topic and return the latest received data. This capability is invaluable for debugging, visualizing data, and allowing non-ROS applications to interact with the robot's internal state.

Ask your AI: Implement a new FastAPI endpoint `/ros2/topic/latest` in `backend/main.py` that takes a `topic_name` query parameter. It should return the simulated JSON response for a given topic. For now, you can hardcode a few responses based on topic names, for example, for `/chatter` and `/scan`. Include appropriate Pydantic models.