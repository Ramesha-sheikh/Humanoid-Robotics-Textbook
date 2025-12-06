---
title: "ROS 2 Kya Hai?"
---

# ROS 2 Kya Hai? (What is ROS 2?)

## 1. Introduction to ROS 2

ROS 2 (Robot Operating System 2) ek open-source framework hai jo robotics software develop karne mein madad karta hai. Yeh robots ke liye applications banane ke liye tools, libraries, aur conventions provide karta hai. ROS 1 ka successor hai, ROS 2 ko modern distributed systems ki zarooraton ko poora karne ke liye design kiya gaya tha, khaas tor par real-time performance, multi-robot coordination, aur security ko behtar banane par focus kiya gaya.

## 2. ROS 2 Ki Zaroorat Kyun? (Why ROS 2?)

ROS 2 ko bohot si wajahat se ROS 1 par tarjeeh di jaati hai:
*   **Distributed Data Service (DDS)**: Data communication ke liye DDS ka istemal karta hai, jo behtar real-time performance, scalability, aur reliability provide karta hai.
*   **Real-time Support**: Embedded systems aur critical applications ke liye behtar real-time capabilities.
*   **Security**: Communication layer par built-in encryption aur authentication, jo production systems ke liye zaroori hai.
*   **Multi-Robot Coordination**: Ek se zyada robots ko ek saath manage karna aasan banata hai.
*   **Cross-Platform Support**: Linux ke ilawa Windows aur macOS par bhi chal sakta hai.
*   **Lifecycle Management**: Nodes ki zindagi (start, stop, pause) ko behtar tareeqay se manage karne ki salahiyat.

## 3. ROS 2 Ke Bunyadi Tasawurat (Fundamental ROS 2 Concepts)

ROS 2 ke bunyadi ajzaa (components) mein shaamil hain:
*   **Nodes**: Yeh independent executable processes hotay hain jo ek khaas kaam perform karte hain (masalan, camera se image lena, motor control karna).
*   **Topics**: Nodes ke darmyan asynchronous data streaming ke liye istemal hotay hain. Ek node data publish karta hai, aur doosra subscribe karta hai.
*   **Messages**: Topics par bheji jaane wali data structures. Har message ka ek khaas type hota hai.
*   **Services**: Request/response communication ke liye istemal hotay hain, jahan ek node request bhejta hai aur doosra uska jawab deta hai. Synchronous communication ke liye behtar hain.
*   **Actions**: Long-running tasks ke liye istemal hotay hain. Yeh services ki tarah hotay hain lekin feedback aur preemption ki salahiyat provide karte hain.
*   **Parameters**: Nodes ke configuration settings jo run-time par change kiye ja sakte hain.

## 4. ROS 2 Ki Tanseekh (Installation)

ROS 2 ki tanseekh (installation) mukhtalif operating systems par ki ja sakti hai, lekin Ubuntu (Linux) par iski installation sab se zyada common aur recommended hai. Official documentation mein detailed steps maujood hain. Misal ke taur par, Foxy Fitzroy ya Galactic Geochelone jaise distributions popular hain.

## 5. Workspace aur Packages (Workspace and Packages)

ROS 2 projects ko **workspaces** mein organize kiya jata hai. Ek workspace mein ek ya zyada **packages** hote hain. Har package mein nodes, launch files, configuration files, aur doosri resources hoti hain.
*   **`colcon`**: ROS 2 ka default build system hai, jo packages ko build, test, aur install karne ke liye istemal hota hai.
*   **Package Structure**: Aam taur par `src`, `include`, `launch`, `config`, aur `test` directories hoti hain.

## 6. Pehla ROS 2 Package Banana (Creating Your First ROS 2 Package)

Python (rclpy) mein ek naya package banane ke liye aap yeh command istemal kar sakte hain:

```bash
# rclpy package banane ke liye, is command ko istemal karein.
# Ye 'my_robot_controller' naam ka ek naya Python package banayega.
ros2 pkg create --build-type ament_python my_robot_controller
```

## 7. ROS 2 Nodes: Publisher

Chalo ek simple publisher node banatay hain jo topic par "Hello ROS 2" messages publish karega.

`my_robot_controller/my_robot_controller/publisher_member_function.py`:

```python
import rclpy # ROS 2 ki Python library. Ye har ROS 2 Python node ke liye zaroori hai.
from rclpy.node import Node # Node class import ki jaati hai.

from std_msgs.msg import String # String message type ko import kiya gaya.


class MinimalPublisher(Node): # Apni class define karein jo Node se inherit karegi.

    def __init__(self): # Constructor method.
        super().__init__('minimal_publisher') # Parent class ke constructor ko call karein aur node ka naam set karein.
        # 'chatter' naam ka topic banao aur String type ke messages publish karo.
        # Queue size 10 ka matlab hai ke zyada se zyada 10 messages store honge agar subscriber slow ho.
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds # Har 0.5 seconds mein message publish hoga.
        self.timer = self.create_timer(timer_period, self.timer_callback) # Timer banao.
        self.i = 0 # Counter ko initialize karo.

    def timer_callback(self): # Timer callback function.
        msg = String() # Naya String message object banao.
        msg.data = 'Hello ROS 2: %d' % self.i # Message data set karo.
        self.publisher_.publish(msg) # Message publish karo.
        # Terminal par message print karo taake pata chale kya publish hua hai.
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1 # Counter ko increment karo.


def main(args=None): # Main function.
    rclpy.init(args=args) # ROS 2 context ko initialize karo.

    minimal_publisher = MinimalPublisher() # Apni publisher node class ka instance banao.

    rclpy.spin(minimal_publisher) # Node ko chalao aur messages process karo jab tak woh band na ho.

    # Node ko destruction ke liye tayyar karo.
    minimal_publisher.destroy_node()
    rclpy.shutdown() # ROS 2 context ko shudown karo.


if __name__ == '__main__':
    main()
```

## 8. ROS 2 Topics: Subscriber

Ab ek subscriber node banatay hain jo `chatter` topic par messages receive karega.

`my_robot_controller/my_robot_controller/subscriber_member_function.py`:

```python
import rclpy # ROS 2 ki Python library.
from rclpy.node import Node # Node class import ki jaati hai.

from std_msgs.msg import String # String message type ko import kiya gaya.


class MinimalSubscriber(Node): # Apni class define karein jo Node se inherit karegi.

    def __init__(self): # Constructor method.
        super().__init__('minimal_subscriber') # Parent class ke constructor ko call karein aur node ka naam set karein.
        # 'chatter' naam ke topic par subscribe karo aur String type ke messages receive karo.
        # QOS profile default rehne do ya explicit set karo.
        self.subscription = self.create_subscription(
            String, # Message type.
            'chatter', # Topic ka naam.
            self.listener_callback, # Callback function jab naya message receive ho.
            10) # Queue size.
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg): # Callback function jab naya message receive ho.
        # Terminal par received message print karo.
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None): # Main function.
    rclpy.init(args=args) # ROS 2 context ko initialize karo.

    minimal_subscriber = MinimalSubscriber() # Apni subscriber node class ka instance banao.

    rclpy.spin(minimal_subscriber) # Node ko chalao aur messages process karo jab tak woh band na ho.

    # Node ko destruction ke liye tayyar karo.
    minimal_subscriber.destroy_node()
    rclpy.shutdown() # ROS 2 context ko shudown karo.


if __name__ == '__main__':
    main()
```

**Packages Build aur Nodes Run Karna (Building Packages and Running Nodes):**

`setup.py` mein entry points add karna zaroori hai. `my_robot_controller/setup.py` ko is tarah edit karein:

```python
from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['%s' % package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_robot_controller.publisher_member_function:main',
            'listener = my_robot_controller.subscriber_member_function:main',
        ],
    },
)
```

Ab workspace ko build karein aur nodes ko chalaein:

```bash
# Workspace ko build karo. Ye saare packages ko compile karega.
colcon build --packages-select my_robot_controller

# Environment setup karo taake naye packages ko system path mein add kiya ja sake.
source install/setup.bash

# Publisher node ko chalao.
ros2 run my_robot_controller talker

# Naya terminal kholein aur environment setup karein (har naye terminal mein zaroori hai).
source install/setup.bash

# Subscriber node ko chalao.
ros2 run my_robot_controller listener
```

## 9. ROS 2 CLI Tools (ROS 2 Command Line Interface Tools)

Kuch ahem `ros2` CLI tools jo aapko kaam aayenge:
*   `ros2 run <package_name> <executable_name>`: Ek node ko run karta hai.
*   `ros2 node list`: Saare running nodes ki list dikhata hai.
*   `ros2 topic list`: Saare active topics ki list dikhata hai.
*   `ros2 topic echo <topic_name>`: Ek topic par publish hone wale messages ko display karta hai.
*   `ros2 pkg list`: Saare available packages ki list dikhata hai.
*   `ros2 interface show <message_type>`: Message type ki definition dikhata hai.

## 10. Ikhtitam (Conclusion)

Is chapter mein humne ROS 2 ke bunyadi tasawurat, iski zaroorat, aur ek simple publisher-subscriber setup banane ka tareeqa seekha. ROS 2 ka istemal karke aap complex robotic systems ko modular aur maintainable tareeqay se develop kar sakte hain. Agle chapters mein hum mazeed gehrai mein jayenge.

## Quiz: Test Your Knowledge

1.  ROS 2 mein nodes ke darmyan asynchronous data streaming ke liye kya istemal hota hai?
    a) Services
    b) Actions
    c) Topics
    d) Parameters

2.  Kon sa ROS 2 component long-running tasks ke liye feedback aur preemption provide karta hai?
    a) Node
    b) Topic
    c) Service
    d) Action

3.  ROS 2 ka default build system kya hai?
    a) Catkin
    b) Make
    c) Colcon
    d) Gmake

4.  `rclpy` mein, ek naya node banane ke liye kis class ko inherit kiya jata hai?
    a) Publisher
    b) Subscriber
    c) Node
    d) Spin

5.  Kon sa command ek running ROS 2 node ki list dikhata hai?
    a) `ros2 topic list`
    b) `ros2 pkg list`
    c) `ros2 node info`
    d) `ros2 node list`
