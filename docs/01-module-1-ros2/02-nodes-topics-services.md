---
title: "ROS 2 Nodes, Topics, aur Services"
---

# ROS 2 Nodes, Topics, aur Services

## 1. ROS 2 Nodes, Topics, aur Services (Introduction)

ROS 2 (Robot Operating System 2) ke computational graph mein nodes, topics, aur services bunyadi ajzaa (fundamental components) hain. Yeh components mil kar ek robotic system ko modular aur flexible tareeqay se chalate hain, jahan har hissa apna kaam alag se karta hai aur doosron ke saath communication karta hai. Is chapter mein hum in teen ahem tasawurat ko tafseel se dekhenge.

## 2. ROS 2 Computational Graph Ka Ta'aruf (Introduction to the Computational Graph)

Computational Graph se murad ROS 2 ke nodes, topics, services, aur actions ka network hai jo run-time par data aur commands ka tabadla karte hain. Yeh ek conceptual diagram hai jo robot ke software components ke darmiyan connections ko zahir karta hai. Iski wajah se system ko debug karna, modify karna, aur scale karna aasan ho jaata hai.

## 3. Nodes: Bunyadi Building Blocks (Nodes: Fundamental Building Blocks)

Node ROS 2 mein ek executable process hai jo ek khaas kaam karta hai. Masalan, ek node camera se images read kar sakta hai, doosra motor control kar sakta hai, aur teesra navigation algorithms chala sakta hai. Nodes autonomous hote hain aur ek doosre se alag chalte hain, lekin topics ya services ke zariye communication karte hain.

### Node Banane ka Example (Example of Creating a Node):

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self): # Constructor method
        super().__init__('my_first_node') # Node ka naam 'my_first_node' rakha
        self.get_logger().info('Mera Pehla ROS 2 Node Shuru Ho Gaya Hai! (My First ROS 2 Node has started!)')

def main(args=None):
    rclpy.init(args=args) # ROS 2 runtime ko initialize karein
    node = MyFirstNode() # MyFirstNode class ka object banaein
    rclpy.spin(node) # Node ko chalate rahein jab tak Ctrl+C na dabaya jaye
    node.destroy_node() # Node ko khatam karein
    rclpy.shutdown() # ROS 2 runtime ko shutdown karein

if __name__ == '__main__':
    main()
```

## 4. Topics: Asynchronous Data Streaming (Topics: Asynchronous Data Streaming)

Topics data stream ke liye istemal hotay hain jahan information ek node se kayi doosre nodes tak real-time mein bheji jaati hai. Yeh publish/subscribe pattern par kaam karte hain. Ek node (publisher) topic par data bhejta hai, aur doosre nodes (subscribers) us data ko receive karte hain.

### Key Characteristics of Topics:
*   **Asynchronous**: Data jab available hoti hai, tabhi bheji jaati hai. Receivers ko fauran data milti hai.
*   **One-to-many/Many-to-many**: Ek publisher kayi subscribers ko data bhej sakta hai, aur kayi publishers ek topic par data bhej sakte hain.
*   **No guarantee of delivery**: Agar subscriber ready na ho to messages miss ho sakte hain (QoS settings par depend karta hai).

## 5. Publishers aur Subscribers (Publishers and Subscribers)

### Publisher Example:

`my_pkg/my_pkg/simple_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # String messages ke liye

class SimplePublisher(Node):
    def __init__(self): # Constructor
        super().__init__('simple_publisher') # Node ka naam set karein
        # 'my_topic' naam ka publisher banao, String type aur queue size 10
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 1.0  # seconds # Har 1 second mein publish karein
        self.timer = self.create_timer(timer_period, self.timer_callback) # Timer set karein
        self.i = 0 # Counter for messages

    def timer_callback(self): # Timer callback function
        msg = String() # Naya String message object
        msg.data = f'Salam ROS 2! Yeh message number {self.i} hai.' # Message data
        self.publisher_.publish(msg) # Message publish karein
        self.get_logger().info(f'Publishing: "{msg.data}"') # Terminal par print karein
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example:

`my_pkg/my_pkg/simple_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self): # Constructor
        super().__init__('simple_subscriber') # Node ka naam set karein
        # 'my_topic' par subscribe karein, String type
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback, # Jab message aaye to ye function call hoga
            10)
        self.subscription  # unused variable warning se bachne ke liye

    def listener_callback(self, msg): # Message receive hone par call hone wala function
        self.get_logger().info(f'Mujhe mila: "{msg.data}"') # Terminal par print karein

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

`setup.py` mein entry points add karein:
```python
# ... (existing setup.py content)
    entry_points={
        'console_scripts': [
            'simple_pub = my_pkg.simple_publisher:main',
            'simple_sub = my_pkg.simple_subscriber:main',
        ],
    },
# ...
```

Run karne ke liye:
```bash
colcon build --packages-select my_pkg
source install/setup.bash
ros2 run my_pkg simple_pub
ros2 run my_pkg simple_sub
```

## 6. Services: Synchronous Request-Response (Services: Synchronous Request-Response)

Services synchronous request-response communication ke liye istemal hotay hain. Jab ek node (client) ko doosre node (server) se koi khaas kaam karwana ho aur jawab ka intezar karna ho, tab services use kiye jaate hain. Masalan, ek robot se uski battery level poochna.

### Key Characteristics of Services:
*   **Synchronous**: Client request bhejta hai aur jawab ka intezar karta hai. Server request ko process karta hai aur response bhejta hai.
*   **One-to-one**: Sirf ek client ek server se baat karta hai ek time par (request-response cycle mein).
*   **Guaranteed delivery**: Request ka jawab milna zaroori hai.

## 7. Service Clients aur Servers (Service Clients and Servers)

Services istemal karne ke liye pehle ek custom service interface (`.srv` file) banani padti hai.

`my_pkg/srv/AddTwoInts.srv`:
```
int64 a
int64 b
---
int64 sum
```

`package.xml` mein dependencies add karein:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

`CMakeLists.txt` mein service build enable karein:
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
)
```

### Service Server Example:

`my_pkg/my_pkg/simple_service_server.py`:

```python
import rclpy
from rclpy.node import Node
from my_pkg.srv import AddTwoInts # Apni custom service import karein

class MinimalService(Node):

    def __init__(self): # Constructor
        super().__init__('minimal_service') # Node ka naam
        # 'add_two_ints' naam ka service server banao
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service Server Shuru Ho Gaya Hai. (Service Server has started.)')

    def add_two_ints_callback(self, request, response): # Service request callback
        response.sum = request.a + request.b # Request ke numbers ko add karo
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Sending back sum={response.sum}')
        return response # Response wapas bhejo

def main(args=None):
    rclpy.init(args=args)
    node = MinimalService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example:

`my_pkg/my_pkg/simple_service_client.py`:

```python
import sys
import rclpy
from rclpy.node import Node
from my_pkg.srv import AddTwoInts # Apni custom service import karein

class MinimalClientAsync(Node):

    def __init__(self): # Constructor
        super().__init__('minimal_client_async') # Node ka naam
        # 'add_two_ints' naam ka service client banao
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0): # Service ke ready hone ka intezar karein
            self.get_logger().info('Service available nahi hai, intezar kar rahe hain... (Service not available, waiting...)')
        self.req = AddTwoInts.Request() # Request object banao

    def send_request(self, a, b): # Request bhejne ka function
        self.req.a = a # Request ke arguments set karein
        self.req.b = b
        self.future = self.cli.call_async(self.req) # Async call karein
        self.get_logger().info(f'Request bheja: a={a}, b={b}')

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    # Command line arguments se values lo
    if len(sys.argv) == 3:
        minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    else:
        minimal_client.get_logger().info('Ghalat istimal: ros2 run my_pkg simple_client <num1> <num2> (Incorrect usage: ...)')
        rclpy.shutdown()
        sys.exit(1)

    while rclpy.ok(): # Jab tak response na mile ya node band na ho
        rclpy.spin_once(minimal_client) # Event loop ko ek baar chalao
        if minimal_client.future.done(): # Agar future complete ho gaya hai
            try:
                response = minimal_client.future.result() # Result lo
            except Exception as e:
                minimal_client.get_logger().error(f'Service call fail hua %r' % (e,)) # Error handling
            else:
                minimal_client.get_logger().info(f'Result: sum = {response.sum}') # Result print karein
            break

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

`setup.py` mein entry points add karein:
```python
# ... (existing setup.py content)
    entry_points={
        'console_scripts': [
            'simple_pub = my_pkg.simple_publisher:main',
            'simple_sub = my_pkg.simple_subscriber:main',
            'service_server = my_pkg.simple_service_server:main',
            'service_client = my_pkg.simple_service_client:main',
        ],
    },
# ...
```

Run karne ke liye:
```bash
colcon build --packages-select my_pkg
source install/setup.bash
ros2 run my_pkg service_server
ros2 run my_pkg service_client 5 3
```

## 8. Parameters (Mukhtasar Ta'aruf) (Parameters: Brief Introduction)

Parameters nodes ke configuration settings hote hain jo run-time par dynamic tareeqay se change kiye ja sakte hain. Yeh nodes ko zyada flexible banate hain. Misal ke taur par, ek camera node ka frame rate parameter ke zariye set kiya ja sakta hai.

## 9. Actions (Mukhtasar Ta'aruf) (Actions: Brief Introduction)

Actions long-running, goal-oriented tasks ke liye istemal hotay hain jahan continuous feedback aur preemption (task ko rokna) ki zaroorat hoti hai. Yeh services se zyada complex hote hain aur navigation ya manipulation jaise tasks ke liye use kiye jaate hain. Ek action client goal bhejta hai, aur action server goal ko process karta hai, feedback deta hai, aur phir result return karta hai.

## 10. Comparison: Topics vs. Services vs. Actions (Comparison Table)

| Feature        | Topics                 | Services                 | Actions                      |
| :------------- | :--------------------- | :----------------------- | :--------------------------- |
| Communication  | Asynchronous streaming | Synchronous request/reply | Asynchronous goal/feedback/result |
| Pattern        | Publish/Subscribe      | Client/Server            | Client/Server                |
| Use Case       | Continuous data flow   | Instant request/response | Long-running tasks, navigation |
| Feedback       | No built-in feedback   | No built-in feedback     | Continuous feedback          |
| Preemption     | No                     | No                       | Yes                          |

## Quiz: Test Your Knowledge

1.  ROS 2 mein kaun sa communication pattern one-to-many data streaming ke liye istemal hota hai?
    a) Services
    b) Actions
    c) Topics
    d) Parameters

2.  Ek ROS 2 Service call ki khaas baat kya hai?
    a) Yeh asynchronous hota hai.
    b) Yeh long-running tasks ke liye hota hai.
    c) Yeh synchronous request-response hota hai.
    d) Ismein continuous feedback hota hai.

3.  Agar aapko robot se uska maujooda battery level poochhna ho aur jawab ka intezar karna ho, to aap kya istemal karenge?
    a) Topic
    b) Service
    c) Action
    d) Parameter

4.  `rclpy` mein ek node create karte waqt, `super().__init__('my_node_name')` call kyun kiya jata hai?
    a) Parent class ke constructor ko call karne aur node ka naam set karne ke liye.
    b) Node ko destroy karne ke liye.
    c) Message publish karne ke liye.
    d) Topic par subscribe karne ke liye.

5.  Kaun sa ROS 2 component navigation jaise long-running tasks ke liye feedback aur preemption provide karta hai?
    a) Topic
    b) Service
    c) Action
    d) Parameter
