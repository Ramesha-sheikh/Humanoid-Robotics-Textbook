## ROS 2 Nodes: The Building Blocks

💬 **Theory Insight: Independent Executables in a ROS Graph**

In the ROS 2 architecture, a *node* is an executable process that performs a specific, atomic task. Think of nodes as individual programs or services that communicate with each other to form a larger robotic application. Each node is typically responsible for a single function, such as reading data from a sensor, controlling a motor, or executing a navigation algorithm.

The philosophy behind nodes is modularity: breaking down complex robotic systems into smaller, manageable, and independently executable units. This approach offers several advantages:

-   **Reusability**: Nodes can be reused in different robotic applications or configurations.
-   **Isolation**: Errors in one node are less likely to crash the entire system.
-   **Concurrency**: Multiple nodes can run simultaneously, often on different computational resources (e.g., CPU cores, GPUs, or even different machines).
-   **Debugging**: It's easier to isolate and debug issues in smaller, focused processes.

Nodes communicate with each other using ROS 2's publish-subscribe mechanism (topics), service calls, and action goals, forming a *ROS graph* where data flows between processing units. Understanding how to create, run, and manage nodes is fundamental to developing any ROS 2 application.

### Structure of a ROS 2 Node

```mermaid
graph TD
    A[ROS 2 Node] --> B{Initialization (rclpy/rclcpp)}
    B --> C[Create Publishers/Subscribers/Clients/Servers]
    C --> D[Main Loop (Spin)]
    D --> E[Process Data & Execute Logic]
    E --> C
    D --> F[Shutdown (on exit)]
```

### Key Node Properties

| Property       | Description                                                               | Importance                                                    |
|----------------|---------------------------------------------------------------------------|---------------------------------------------------------------|
| **Name**       | Unique identifier for the node instance within the ROS graph              | Essential for addressing nodes, debugging, and configuration  |
| **Namespace**  | Logical grouping for nodes, topics, and services                          | Prevents naming conflicts in large systems, aids organization |
| **Parameters** | Configurable values that modify a node's behavior without recompilation   | Flexibility, adaptability to different hardware/scenarios     |
| **Lifecycle**  | Managed states (unconfigured, inactive, active) for robust behavior       | Crucial for mission-critical applications, graceful startup/shutdown |

🎓 **Key Insight: The Importance of Node Naming and Namespaces**

In a complex ROS 2 application, particularly for humanoid robots with many sensors, actuators, and processing units, managing a multitude of nodes can quickly become challenging. The concepts of *node naming* and *namespaces* are critical for maintaining order, preventing conflicts, and ensuring proper communication.

Every node should have a unique name within its namespace. This allows other nodes to target it for services or actions, and makes debugging straightforward (e.g., using `ros2 node info /my_robot/camer-node`). Namespaces provide a hierarchical structure, much like directories in a file system (e.g., `/my_robot/sensors/lidar_node`). This prevents name collisions when integrating components from different developers or reusing generic nodes. Properly structured namespaces also allow for easy remapping of topics and services, making a robot configuration highly flexible.

For example, if you have two identical humanoid robots, you can launch the same set of nodes for each robot, but put them in different namespaces (e.g., `/robot1/camer-node` and `/robot2/camer-node`), allowing them to operate independently without their topics or services conflicting.

### Code Example: Simple ROS 2 Python Node (Publisher)

This Python code defines a basic ROS 2 node that publishes a string message to a topic. This requires a ROS 2 environment (e.g., Iron) to be sourced.

```python
# File: ros2_ws/src/simple_publisher_pkg/simple_publisher_pkg/publisher_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher_node') # Initialize the node with a name
        self.publisher_ = self.create_publisher(String, 'chatter', 10) # Create a publisher
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info(f"Simple Publisher Node started, publishing to 'chatter' topic.")

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2 from Python: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"') # Log the published message
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2 Python client library

    simple_publisher = SimplePublisher()

    rclpy.spin(simple_publisher) # Keep node alive until Ctrl+C

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this code:
1.  Save it as `ros2_ws/src/simple_publisher_pkg/simple_publisher_pkg/publisher_node.py` (you'll need to create the package structure).
2.  Build your ROS 2 workspace (after setting up `colcon_build`).
3.  Source your ROS 2 environment.
4.  Run: `ros2 run simple_publisher_pkg publisher_node`

This basic node demonstrates how to initialize `rclpy`, create a publisher, and publish messages periodically. This will be foundational for making our robot components communicate.

🤝 **Practice: Listing Active ROS 2 Nodes via FastAPI**

To effectively manage and monitor our Physical AI system, especially when integrating with VLA models, our FastAPI backend needs to be able to query the ROS 2 graph. This practice involves conceptually extending the FastAPI backend (`backend/main.py`) to list currently active ROS 2 nodes in a simulated environment. This allows the backend to assess the operational status of the robot's software components.

### `curl` Example: Listing Simulated ROS 2 Nodes

Assume our FastAPI backend has an endpoint `/ros2/nodes` that, when queried, returns a list of names of currently active ROS 2 nodes.

```bash
# Placeholder curl command - replace with actual FastAPI endpoint once ready
# Ensure your FastAPI backend (backend/main.py) is running (e.g., uvicorn main:app --reload)

curl -X GET \\
  http://127.0.0.1:8000/ros2/nodes \\
  -H "Content-Type: application/json"
```

**Expected (Simulated) FastAPI Response:**

```json
{
  "status": "success",
  "active_nodes": [
    "/simple_publisher_node",
    "/camera_driver",
    "/robot_arm_controller"
  ],
  "timestamp": "2025-12-07T14:30:00Z"
}
```

This `curl` command simulates a request to our FastAPI backend for information about the ROS 2 graph. In a real implementation, the FastAPI endpoint would use `rclpy.node.get_node_names()` or similar introspection functions to discover and report the active nodes. This capability is essential for diagnostics, orchestrating robot behaviors, and ensuring all necessary components are operational before issuing complex commands.

Ask your AI: Implement a new FastAPI endpoint `/ros2/nodes` in `backend/main.py` that returns the simulated JSON response, representing a list of active ROS 2 nodes. Include appropriate Pydantic models for the response.