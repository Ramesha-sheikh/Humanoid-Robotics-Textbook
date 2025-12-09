# ROS 2: Services & Actions

## Services: Synchronous Request/Response

While topics provide an asynchronous, one-to-many communication stream, **services** in ROS 2 offer a synchronous, one-to-one request/response mechanism. This is ideal for tasks where a node needs to request a specific computation or data from another node and wait for the result before proceeding.

### Key Characteristics of Services:
-   **Synchronous:** The client blocks and waits for the server's response.
-   **One-to-one:** A single client sends a request to a single server.
-   **Service Type:** Defined by `.srv` files, specifying the structure of both the request and the response messages.

### How Services Work:
1.  A **service server** node advertises a service under a unique name.
2.  A **service client** node creates a request message and sends it to the server.
3.  The server processes the request, performs a computation, and generates a response.
4.  The server sends the response back to the client.
5.  The client receives the response and unblocks, continuing its execution.

**Creating a Service Server (Python - `rclpy`):**

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Custom service type

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        # Create a service with type AddTwoInts, name 'add_two_ints', and callback function
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('AddTwoInts Service Server has started!')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b # Perform the addition
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    server = AddTwoIntsServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Creating a Service Client (Python - `rclpy`):**

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service 'add_two_ints' not available, waiting...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        # Call the service asynchronously and wait for the result
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: ros2 run <package_name> <node_name> <a> <b>')
        sys.exit(1)

    client = AddTwoIntsClient()
    response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    client.get_logger().info(f'Result of AddTwoInts: {client.req.a} + {client.req.b} = {response.sum}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions: Goal-Oriented, Long-Running Tasks

**Actions** are a higher-level communication mechanism designed for long-running, goal-oriented tasks that provide periodic feedback and can be preempted (canceled). They are commonly used for tasks like navigating to a goal, manipulating an object, or executing a complex sequence of movements.

### Key Characteristics of Actions:
-   **Asynchronous:** Client sends a goal and can continue other tasks while waiting for feedback and result.
-   **Goal:** The desired state or task to be achieved.
-   **Feedback:** Intermediate updates on the progress of the action.
-   **Result:** The final outcome of the action upon completion.
-   **Preemptable:** A client can cancel an ongoing action.
-   **Action Type:** Defined by `.action` files, specifying the structure of the goal, result, and feedback messages.

### How Actions Work:
1.  An **action server** node advertises an action.
2.  An **action client** node sends a goal to the server.
3.  The server begins executing the goal, sending **feedback** messages periodically to the client.
4.  The client can monitor the feedback, or choose to send a request to **cancel** the goal.
5.  Upon completion (or cancellation), the server sends a final **result** message to the client.

**Analogy:** A service is like ordering a coffee and waiting at the counter. An action is like ordering a pizza for delivery; you get updates on its status and can call to cancel if needed.

### Roman Urdu Explanation:

`Services woh communication hain jahan ek node doosre se kaam karwata hai aur jawab ka intezar karta hai. Jaise tumne kisi se sawal poocha aur jawab milne tak ruka. Jabke Actions lambe kaam ke liye hain, jismein tum kaam karne wale ko goal batate ho, woh tumhe progress batata rehta hai (feedback) aur tum us kaam ko beech mein rok bhi sakte ho (preempt).`

### Multiple Choice Questions (MCQs):

1.  **Which ROS 2 communication method is synchronous and typically used for short-duration tasks?**
    a) Topics
    b) Services
    c) Actions
    d) Parameters
    *Correct Answer: b) Services*

2.  **What is a key feature that Actions provide, which Services do not?**
    a) One-to-one communication
    b) Asynchronous interaction
    c) Message types
    d) Client-server model
    *Correct Answer: b) Asynchronous interaction*

3.  **In an Action, what provides updates on the progress of a long-running task?**
    a) Goal
    b) Result
    c) Feedback
    d) Request
    *Correct Answer: c) Feedback*

4.  **Which file extension is used to define the request and response structure for a ROS 2 Service?**
    a) `.msg`
    b) `.action`
    c) `.srv`
    d) `.yaml`
    *Correct Answer: c) `.srv`*

5.  **If you need to cancel an ongoing task in ROS 2, which communication mechanism would be most appropriate?**
    a) Topics
    b) Services
    c) Actions
    d) Parameters
    *Correct Answer: c) Actions*

### Further Reading:
- [ROS 2 Services (Conceptual)](https://docs.ros.org/en/humble/Concepts/Basic-Concepts/About-Services.html)
- [ROS 2 Actions (Conceptual)](https://docs.ros.org/en/humble/Concepts/Basic-Concepts/About-Actions.html)
