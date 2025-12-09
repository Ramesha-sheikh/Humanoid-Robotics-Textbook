## Advanced ROS 2 Topics and QoS

💬 **Theory Insight: Beyond Basic Publish-Subscribe**

In Week 3, we introduced ROS 2 topics as the primary mechanism for real-time data streaming. Now, we'll dive deeper into advanced aspects of topics, particularly focusing on the various **Quality of Service (QoS)** policies that allow fine-grained control over message delivery. Understanding and correctly configuring QoS is critical for building robust, performant, and reliable robotic systems, especially those with real-time requirements or operating in challenging network conditions.

QoS profiles consist of several policies that define the behavior of publishers and subscribers. These policies ensure that data communication meets the specific needs of an application, whether it's high-frequency sensor data, critical control commands, or debugging information. Key QoS policies include:

-   **Reliability**: `Reliable` (guaranteed delivery, retransmissions) vs. `Best Effort` (messages may be dropped for timeliness).
-   **Durability**: `Transient Local` (new subscribers receive some history) vs. `Volatile` (only new messages).
-   **History**: `Keep Last N` (store last N messages) vs. `Keep All` (store all messages).
-   **Liveliness**: How long a publisher or subscriber is considered 'alive' before being declared dead.
-   **Deadline**: The expected maximum time between messages.
-   **Lifespan**: How long a message is valid.

Matching QoS profiles between publishers and subscribers is essential for successful communication. If profiles are incompatible, communication will fail. This granularity enables developers to optimize network usage, manage latency, and ensure data integrity based on the unique characteristics of each data stream.

### QoS Policies in Action

```mermaid
graph TD
    Publisher[Publisher Node] -- Publishes data with QoS --> TopicA[Topic: /sensor_data]
    TopicA --> Subscriber1[Subscriber Node 1]
    TopicA --> Subscriber2[Subscriber Node 2]

    subgraph QoS Profile 1
        P1Reliability[Reliability: BEST_EFFORT]
        P1History[History: KEEP_LAST (1)]
    end
    subgraph QoS Profile 2
        P2Reliability[Reliability: RELIABLE]
        P2History[History: KEEP_ALL]
    end

    Publisher --- P1Reliability
    Publisher --- P1History
    Subscriber1 --- P1Reliability
    Subscriber1 --- P1History

    Subscriber2 --- P2Reliability
    Subscriber2 --- P2History
    style Subscriber1 fill:#f9f,stroke:#333,stroke-width:2px
    style Subscriber2 fill:#ccf,stroke:#333,stroke-width:2px
```

### Common QoS Scenarios in Robotics

| Data Type          | Typical QoS Profile                                                     | Rationale                                                                        |
|--------------------|-------------------------------------------------------------------------|----------------------------------------------------------------------------------|
| **Sensor Streams** | Best Effort, Keep Last (1), Volatile                                    | Prioritizes freshness over guaranteed delivery; older data is quickly outdated. |
| **Control Commands** | Reliable, Keep Last (1), Volatile, short Deadline                     | Commands must not be lost; only the latest command matters for control.         |
| **Map Data**       | Reliable, Transient Local, Keep All (or large N)                        | New subscribers need the full map history; map updates are critical.            |
| **Parameter Updates** | Reliable, Transient Local, Keep Last (1)                                | Configuration changes must be guaranteed; late joiners need latest config.      |

🎓 **Key Insight: The Interplay of DDS, RMW, and QoS for Real-time Performance**

The power of ROS 2's communication lies not just in its publish-subscribe model, but in the intricate interplay between the **Data Distribution Service (DDS)** standard, the **ROS Middleware (RMW)** abstraction layer, and the **QoS policies**. DDS provides the underlying real-time communication capabilities, with various implementations (Fast RTPS, Cyclone DDS, RTI Connext) offering different performance characteristics.

RMW acts as a crucial bridge, allowing ROS 2 applications to select and utilize different DDS implementations seamlessly without changing their core code. This middleware-agnostic approach is vital for critical applications like humanoid robotics, where specific DDS characteristics (e.g., latency, throughput, memory footprint) might be optimized for different hardware or real-time operating systems (RTOS). By carefully selecting the RMW implementation and tuning the QoS policies, developers can tailor ROS 2 communication to achieve predictable, high-performance, and reliable data exchange, which is absolutely essential for the safety, stability, and responsiveness of an advanced humanoid robot.

### Code Example: ROS 2 Publisher with Custom QoS (Python)

This example demonstrates creating a ROS 2 publisher with a custom QoS profile, specifically setting `Reliability` to `RMW_QOS_POLICY_RELIABILITY_RELIABLE` and `History` to `RMW_QOS_POLICY_HISTORY_KEEP_LAST` with a depth of 1. This ensures that every message is delivered and only the latest message is kept.

```python
# File: ros2_ws/src/custom_qos_pkg/custom_qos_pkg/publisher_qos_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class CustomQoSPublisher(Node):

    def __init__(self):
        super().__init__('custom_qos_publisher_node')

        # Define a custom QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Ensure message delivery
            history=HistoryPolicy.KEEP_LAST,       # Only keep the latest message
            depth=1                                # Keep only 1 message in history
        )

        self.publisher_ = self.create_publisher(String, 'qos_chatter', qos_profile)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info(f"Custom QoS Publisher Node started, publishing to 'qos_chatter' topic with RELIABLE QoS.")

    def timer_callback(self):
        msg = String()
        msg.data = f'Reliable Message {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    custom_qos_publisher = CustomQoSPublisher()
    rclpy.spin(custom_qos_publisher)
    custom_qos_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this code, you'd set up a ROS 2 package, build it, source your environment, and then run `ros2 run custom_qos_pkg publisher_qos_node`. A corresponding subscriber (not shown here) would need to use a compatible QoS profile to receive messages.

🤝 **Practice: Dynamically Changing QoS via FastAPI**

For advanced robotic systems, the ability to dynamically adjust QoS settings for topics can be crucial for adapting to changing network conditions, prioritizing critical data, or reconfiguring communication during runtime. Our FastAPI backend can serve as an interface to demonstrate this conceptual dynamic QoS adjustment. This practice involves simulating a `curl` command to tell our backend to conceptually change the QoS profile for a given topic.

### `curl` Example: Requesting QoS Change

Assume our FastAPI backend (`backend/main.py`) has an endpoint `/ros2/topic/set_qos` that takes a topic name and new QoS parameters (e.g., reliability, history policy) in a JSON payload.

```bash
# Placeholder curl command - replace with actual FastAPI endpoint once ready
# Ensure your FastAPI backend (backend/main.py) is running (e.g., uvicorn main:app --reload)

curl -X POST \\
  http://127.0.0.1:8000/ros2/topic/set_qos \\
  -H "Content-Type: application/json" \\
  -d '{
    "topic_name": "/sensor_data",
    "reliability": "best_effort",
    "history_policy": "keep_last",
    "history_depth": 5
  }'
```

**Expected (Simulated) FastAPI Response:**

```json
{
  "status": "success",
  "message": "QoS settings for /sensor_data conceptually updated.",
  "applied_qos": {"reliability": "best_effort", "history_policy": "keep_last", "history_depth": 5}
}
```

This `curl` command simulates a request to reconfigure topic communication. In a full implementation, the FastAPI backend would interact with a ROS 2 `LifecycleNode` or a specialized ROS 2 tool to apply these QoS changes. This capability highlights how high-level AI or human operators can manage the low-level communication fabric of a Physical AI system, ensuring optimal performance and reliability under varying operational demands.

Ask your AI: Implement a new FastAPI endpoint `/ros2/topic/set_qos` in `backend/main.py` that accepts a `topic_name` and QoS parameters via a JSON payload. It should log the requested changes and return the simulated JSON response, including appropriate Pydantic models for request and response validation.