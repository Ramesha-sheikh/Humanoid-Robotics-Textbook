# ROS 2 Nodes, Topics, aur Services

Yeh chapter ROS 2 ke bunyadi computational graph concepts ko samjhayega: Nodes, Topics, aur Services. Hum inke istemal, kaam karne ke tareeqay, aur rclpy mein inko implement karne ke tareeqay ko detail mein dekhenge.

## 1. ROS 2 Computational Graph Ka Ta'aruf

ROS 2 ek distributed system hai jahan mukhtalif processes (nodes) apas mein communication karte hain. Yeh communication ek computational graph banata hai, jismein nodes, topics, services, aur actions shamil hain. Is graph ka maqsad robotics applications ko modular aur distributed tareeqay se banana hai.

## 2. Nodes: Bunyadi Building Blocks

**Node kya hai?**
Ek node ek executable process hai jo ROS 2 computational graph ka hissa hota hai. Har node ek khaas zimmedari (task) ko pura karta hai, maslan:
- Sensor data ko read karna
- Motor ko control karna
- Path planning karna
- Image processing karna

Nodes independent hote hain aur apas mein messages ke zariye rabta karte hain.

**Example: rclpy mein Node banana**

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node') # Node ka naam set karein (apna pehla node)
        self.get_logger().info('Hello from my first ROS 2 node!') # Console par message print karein

def main(args=None):
    rclpy.init(args=args) # rclpy library ko initialize karein
    node = MyFirstNode() # Apna node object banayein
    rclpy.spin(node) # Node ko chalate rahein jab tak usko band na kiya jaye
    node.destroy_node() # Node ko saaf karein
    rclpy.shutdown() # rclpy library ko band karein

if __name__ == '__main__':
    main()
```

## 3. Topics: Asynchronous Data Streaming

**Topic kya hai?**
Topics ek asynchronous communication mechanism hain jahan nodes data stream karte hain. Ek node ek topic par data "publish" karta hai, aur doosre nodes us topic ko "subscribe" kar ke data receive karte hain. Yeh one-to-many communication ke liye behtareen hai.

**Example: Topics ke aham khubiyan**
-   **Anonymity:** Publishers aur Subscribers ko ek doosre ka pata hona zaroori nahi. Sirf topic name ka pata hona chahiye.
-   **Decoupling:** Nodes apas mein loosely coupled hote hain, jo system ko robust banata hai.
-   **Real-time data:** Sensor readings, robot odometry, image frames jaisi real-time data ke liye ideal.

## 4. Publishers aur Subscribers

**Publisher:** Woh node jo data create karta hai aur usko ek topic par send karta hai.
**Subscriber:** Woh node jo ek topic se data receive karta hai.

**Example: rclpy mein Publisher aur Subscriber**

Pehle, ek publisher node banate hain (`simple_publisher.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # String message type istemal karein

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher') # Node ka naam 'simple_publisher' set karein
        self.publisher_ = self.create_publisher(String, 'my_topic', 10) # 'my_topic' par String messages publish karein
        self.timer = self.create_timer(0.5, self.timer_callback) # Har 0.5 second par callback function chalayein
        self.i = 0 # Counter shuru karein

    def timer_callback(self):
        msg = String() # Naya String message banayein
        msg.data = f'Hello ROS 2! {self.i}' # Message data set karein
        self.publisher_.publish(msg) # Message publish karein
        self.get_logger().info(f'Publishing: "{msg.data}"') # Console par publish kiya hua data dikhayein
        self.i += 1 # Counter barhayein

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Phir, ek subscriber node banate hain (`simple_subscriber.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # String message type istemal karein

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber') # Node ka naam 'simple_subscriber' set karein
        self.subscription = self.create_subscription(
            String,
            'my_topic', # 'my_topic' ko subscribe karein
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"') # Receive kiya hua data console par dikhayein

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. Services: Synchronous Request-Response

**Service kya hai?**
Services ek synchronous communication mechanism hain jahan ek node (client) ek doosre node (server) se request karta hai aur response ka intezar karta hai. Yeh request-response pattern ke liye use hota hai, maslan:
- Calculation request karna
- Configuration change karna
- Ek specific action trigger karna

**Example: Services ke aham khubiyan**
-   **Synchronous:** Client request bhejta hai aur response milne tak block rehta hai.
-   **One-to-one communication:** Ek client ek server se baat karta hai.
-   **Atomic operations:** Aise tasks ke liye jinka poora hona zaroori hai aur jinke liye ek reply ki zaroorat hoti hai.

## 6. Service Clients aur Servers

**Service Server:** Woh node jo requests receive karta hai, unko process karta hai, aur response send karta hai.
**Service Client:** Woh node jo request send karta hai aur response ka intezar karta hai.

**Example: rclpy mein Service Server aur Client**

Pehle, ek service interface banate hain. Ek naya package create karna hoga (`my_custom_interfaces`).
`my_custom_interfaces/srv/AddTwoInts.srv` file banayein:

```
int64 a
int64 b
---
int64 sum
```

Apne `setup.py` aur `package.xml` files ko update karein `my_custom_interfaces` package mein takay yeh service ko build kar sake.

Phir, ek service server node banate hain (`simple_service_server.py`):

```python
import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import AddTwoInts # Apni custom service import karein

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server') # Node ka naam 'simple_service_server' set karein
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback) # Service banayein
        self.get_logger().info('Service server ready.') # Service server tayar hai

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b # Do integers ko jama karein
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}') # Received request dikhayein
        self.get_logger().info(f'Sending response: sum={response.sum}') # Sent response dikhayein
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    simple_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Phir, ek service client node banate hain (`simple_service_client.py`):

```python
import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import AddTwoInts # Apni custom service import karein
import sys

class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client') # Node ka naam 'simple_service_client' set karein
        self.cli = self.create_client(AddTwoInts, 'add_two_ints') # Service client banayein
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...') # Service available nahi, intezar kar rahe hain
        self.req = AddTwoInts.Request() # Request object banayein

    def send_request(self, a, b):
        self.req.a = a # 'a' value set karein
        self.req.b = b # 'b' value set karein
        self.future = self.cli.call_async(self.req) # Async call karein

def main(args=None):
    rclpy.init(args=args)
    simple_service_client = SimpleServiceClient()

    if len(sys.argv) != 3:
        simple_service_client.get_logger().info('Usage: ros2 run <package_name> simple_service_client <arg1> <arg2>') # Usage guide dikhayein
        simple_service_client.destroy_node()
        rclpy.shutdown()
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])
    simple_service_client.send_request(a, b) # Request send karein

    while rclpy.ok():
        rclpy.spin_once(simple_service_client)
        if simple_service_client.future.done():
            try:
                response = simple_service_client.future.result()
            except Exception as e:
                simple_service_client.get_logger().error(f'Service call failed: {e}') # Service call mein ghalti
            else:
                simple_service_client.get_logger().info(
                    f'Result of add_two_ints: for {a} + {b} = {response.sum}') # Result dikhayein
            break

    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 7. Parameters (Mukhtasar Ta'aruf)

Parameters nodes ko dynamic configuration provide karte hain. Har node ke paas apne parameters ho sakte hain jinhe runtime par modify kiya ja sakta hai. Yeh system ke behavior ko adjust karne mein madad karte hain.

## 8. Actions (Mukhtasar Ta'aruf)

Actions ek high-level communication type hain jo long-running goals (maslan, robot ko ek specific location par move karna) ko handle karte hain. Yeh topics aur services ka combination hain, jismein feedback, goal, aur result shamil hote hain.

## 9. Comparison: Topics vs. Services vs. Actions

| Feature     | Topics                              | Services                            | Actions                                      |
| :---------- | :---------------------------------- | :---------------------------------- | :------------------------------------------- |
| **Pattern** | Asynchronous streaming (pub/sub)    | Synchronous request/response        | Asynchronous goal/feedback/result            |
| **Use Case**| Continuous data (sensors, odometry) | Instant requests (calculations, config) | Long-running tasks (navigation, manipulation) |
| **Blocking**| No                                  | Yes (client waits for response)     | No (client gets feedback, can cancel goal)   |
| **Feedback**| No (only stream data)               | No (only single response)           | Yes (continuous feedback during execution)   |
| **Best For**| Data broadcasting                   | Atomic operations, queries          | Complex, long-duration tasks                 |

## 10. rclpy Code Examples Summary

Upar diye gaye sections mein humne rclpy ka istemal karte hue nodes, topics (publishers aur subscribers), aur services (clients aur servers) ko banaya aur unke working principle ko samjha. In examples ko chalane ke liye aapko ROS 2 environment setup karna hoga aur Python scripts ko `colcon build` ke baad `ros2 run` se execute karna hoga. Custom service ke liye `my_custom_interfaces` package ko bhi build karna zaroori hai.

## Multiple Choice Questions (MCQs)

**1. ROS 2 mein asynchronous data streaming ke liye kaunsa communication mechanism istemal hota hai?**
   a) Services
   b) Actions
   c) Topics
   d) Parameters

**2. Ek node jo data create karta hai aur usko ek topic par send karta hai, usko kya kehte hain?**
   a) Subscriber
   b) Server
   c) Client
   d) Publisher

**3. Services mein client aur server ke darmiyan communication kis tarah ka hota hai?**
   a) Asynchronous streaming
   b) Synchronous request-response
   c) Continuous feedback
   d) One-way broadcast

**4. Long-running goals jaise robot navigation ke liye ROS 2 mein kaunsa communication type behtar hai?**
   a) Topics
   b) Services
   c) Actions
   d) Parameters

**5. `rclpy.spin(node)` function ka bunyadi maqsad kya hai?**
   a) Node ko initialize karna
   b) Node ko band karna
   c) Node ko events process karne ke liye chalate rehna
   d) Node ka naam set karna
