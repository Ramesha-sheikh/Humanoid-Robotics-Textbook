# ROS 2: Services

## Understanding ROS 2 Services

Services in ROS 2 provide a way for nodes to communicate with each other in a synchronous request/response pattern. Unlike topics, where data flows continuously, services are used for calls that require a direct response.

### Key Concepts:
- **Service Server:** A node that offers a service, listens for requests, processes them, and sends back a response.
- **Service Client:** A node that sends a request to a service server and waits for a response.
- **Service Type:** Defines the structure of the request and response messages for a particular service (e.g., `example_interfaces/AddTwoInts`). Custom service types are defined using `.srv` files.

### How Services Work:
1. A service server advertises a service under a unique name.
2. A service client creates a request message and sends it to the server.
3. The server receives the request, performs an operation (e.g., calculation, data retrieval), and generates a response.
4. The server sends the response back to the client.
5. The client receives the response and continues its execution. The client is blocked until it receives a response or a timeout occurs.

### Creating a Simple ROS 2 Service (Python Example):

#### Service Server Node:

```python
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Service Client Node:

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' % (
            minimal_client.req.a, minimal_client.req.b, response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Further Reading:
- [ROS 2 Services (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Writing-A-Simple-Cpp-Service-And-Client.html)
- [ROS 2 Services (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Writing-A-Simple-Python-Service-And-Client.html)
