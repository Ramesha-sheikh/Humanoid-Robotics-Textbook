# 01-ROS2 Introduction

## 1. Introduction

Robot Operating System 2 (ROS2) aik open-source framework hai jo robot application development ko aasan banata hai. Yeh ek set of tools, libraries, aur conventions provide karta hai jo complex robotic systems banane mein madad karte hain. ROS2 ko pehle ROS (Robot Operating System) ki limitations ko overcome karne ke liye design kiya gaya tha, khaas taur par real-time performance, multi-robot systems, aur embedded platforms ke liye.

## 2. Concepts

ROS2 mein kuch bunyadi concepts hain jo samajhna zaroori hai:

*   **Nodes (Nodes):** Yeh executable processes hote hain jo robotic system mein specific tasks perform karte hain. Jaise, ek node camera data read kar sakta hai, aur doosra node navigation calculation kar sakta hai.
*   **Topics (Topics):** Nodes ke darmiyan data exchange karne ka ek tareeqa hai. Ek node data publish karta hai ek specific topic par, aur doosra node us topic ko subscribe karke data receive karta hai. Yeh one-to-many communication hai.
*   **Messages (Messages):** Topics par send kiye jaane wale data structures hain. Har topic ka apna message type hota hai jo us data ka format define karta hai.
*   **Services (Services):** Request/response communication mechanism hai do nodes ke darmiyan. Ek node request bhejta hai, aur doosra node response return karta hai. Yeh synchronous communication hai.
*   **Actions (Actions):** Long-running tasks ke liye use hota hai. Actions services ki tarah hain lekin feedback provide karte hain jab tak task complete na ho jaye, aur goal ko cancel bhi kiya ja sakta hai.
*   **ROS Graph (ROS Graph):** Yeh sare nodes, topics, services, aur actions ka collection hai jo real-time mein ek saath kaam karte hain.
*   **DDS (Data Distribution Service):** ROS2 ke underling transport layer hai jo nodes ke darmiyan reliable aur efficient communication enable karta hai.

## 3. Setup

ROS2 ko setup karna bohot straightforward hai. Yahan basic steps diye gaye hain (assuming Ubuntu ya similar Linux distribution):

1.  **Repository Setup:** ROS2 repositories ko system mein add karein.
    ```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
2.  **Install ROS2 Packages:** Full Desktop installation recommend ki jaati hai.
    ```bash
    sudo apt update
    sudo apt upgrade -y
    sudo apt install ros-humble-desktop -y # Replace humble with your ROS2 distribution (e.g., iron, jazzy)
    ```
3.  **Source the Setup Script:** Har naye terminal session mein ROS2 environment ko source karna zaroori hai.
    ```bash
    source /opt/ros/humble/setup.bash # Replace humble with your ROS2 distribution
    ```
    Permanent source ke liye `~/.bashrc` mein add karein:
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```

## 4. Code Examples

Yahan `rclpy` (ROS2 Client Library for Python) ka istemal karte hue ek simple publisher aur subscriber node ke examples diye gaye hain.

### Publisher Node (`simple_publisher.py`)

```python
import rclpy # rclpy library ko import kar rahe hain. Yeh ROS2 Python clients ke liye hai.
from rclpy.node import Node # Node class ko import kar rahe hain jo sabhi ROS2 nodes ka base hai.

from std_msgs.msg import String # String message type ko import kar rahe hain. Hum yahi type publish karenge.

class SimplePublisher(Node): # SimplePublisher naam ka ek naya class bana rahe hain jo Node class se inherit karta hai.

    def __init__(self): # Constructor method hai. Jab object banega, yeh chalega.
        super().__init__('simple_publisher') # Parent (Node) class ke constructor ko call kar rahe hain aur node ka naam 'simple_publisher' set kar rahe hain.
        self.publisher_ = self.create_publisher(String, 'my_topic', 10) # Ek publisher bana rahe hain.
        # String message type hoga, 'my_topic' uska naam hoga, aur queue size 10 hai.
        timer_period = 0.5  # seconds # Kitni der baad message publish karna hai, woh set kar rahe hain (0.5 seconds).
        self.timer = self.create_timer(timer_period, self.timer_callback) # Ek timer bana rahe hain. Har 0.5 second baad timer_callback function chalega.
        self.i = 0 # Ek counter variable bana rahe hain.

    def timer_callback(self): # Yeh function har 0.5 second baad chalega.
        msg = String() # Naya String message object bana rahe hain.
        msg.data = 'Hello ROS2: %d' % self.i # Message ka data set kar rahe hain.
        self.publisher_.publish(msg) # Message ko 'my_topic' par publish kar rahe hain.
        self.get_logger().info('Publishing: "%s"' % msg.data) # Console par message print kar rahe hain.
        self.i += 1 # Counter ko ek se badha rahe hain.


def main(args=None): # main function hai jahan se program shuru hoga.
    rclpy.init(args=args) # ROS2 client library ko initialize kar rahe hain.

    simple_publisher = SimplePublisher() # SimplePublisher class ka object bana rahe hain.

    rclpy.spin(simple_publisher) # Node ko chalate rehne ke liye spin() call kar rahe hain. Yeh callbacks ko process karta hai.

    # Destroy the node explicitly
    # Niche wale do lines tab chalenge jab rclpy.spin() terminate hoga (Ctrl+C dabane par).
    simple_publisher.destroy_node() # Node ko destroy kar rahe hain.
    rclpy.shutdown() # ROS2 client library ko shutdown kar rahe hain.


if __name__ == '__main__': # Agar yeh script directly run ho raha hai, toh main() function ko call karein.
    main()
```

### Subscriber Node (`simple_subscriber.py`)

```python
import rclpy # rclpy library ko import kar rahe hain.
from rclpy.node import Node # Node class ko import kar rahe hain.

from std_msgs.msg import String # String message type ko import kar rahe hain. Hum yahi type receive karenge.

class SimpleSubscriber(Node): # SimpleSubscriber naam ka class bana rahe hain jo Node class se inherit karta hai.

    def __init__(self): # Constructor method.
        super().__init__('simple_subscriber') # Parent class ke constructor ko call kar rahe hain aur node ka naam 'simple_subscriber' set kar rahe hain.
        self.subscription = self.create_subscription(
            String, # Message type String hoga.
            'my_topic', # Jis topic ko subscribe karna hai uska naam 'my_topic' hai.
            self.listener_callback, # Jab message receive hoga, toh listener_callback function chalega.
            10) # Queue size 10 hai.
        self.subscription  # prevent unused variable warning # Warning se bachne ke liye yeh line add ki hai.

    def listener_callback(self, msg): # Yeh function tab chalega jab 'my_topic' par koi message receive hoga.
        self.get_logger().info('I heard: "%s"' % msg.data) # Receive kiye gaye message ko console par print kar rahe hain.


def main(args=None): # main function.
    rclpy.init(args=args) # ROS2 client library ko initialize kar rahe hain.

    simple_subscriber = SimpleSubscriber() # SimpleSubscriber class ka object bana rahe hain.

    rclpy.spin(simple_subscriber) # Node ko chalate rehne ke liye spin() call kar rahe hain.

    # Destroy the node explicitly
    simple_subscriber.destroy_node() # Node ko destroy kar rahe hain.
    rclpy.shutdown() # ROS2 client library ko shutdown kar rahe hain.


if __name__ == '__main__': # Agar yeh script directly run ho raha hai, toh main() function ko call karein.
    main()
```

## 5. Explanation

**Publisher Node (`simple_publisher.py`):**
1.  **Imports:** `rclpy` aur `Node` base class, aur `std_msgs.msg.String` message type import kiye gaye hain.
2.  **Class `SimplePublisher`:** `Node` class se inherit karta hai.
    *   `__init__` constructor mein, `super().__init__('simple_publisher')` se node initialize hota hai.
    *   `create_publisher` method ka use karke ek publisher banaya gaya hai jo `String` type ke messages `my_topic` par publish karega. `10` queue size hai.
    *   `create_timer` method ek timer banata hai jo har `0.5` seconds par `timer_callback` method ko execute karega.
3.  **`timer_callback`:** Har `0.5` seconds par ek naya `String` message banata hai, usmein data set karta hai, aur use `publisher_.publish(msg)` ke zariye `my_topic` par send karta hai. Console par bhi print karta hai.
4.  **`main` function:**
    *   `rclpy.init()` ROS2 client library ko initialize karta hai.
    *   `SimplePublisher()` object banaya jaata hai.
    *   `rclpy.spin(simple_publisher)` node ko infinite loop mein chalata hai, jo callbacks (jaise `timer_callback`) ko process karta hai.
    *   Jab `spin` terminate hota hai (Ctrl+C), `destroy_node()` aur `rclpy.shutdown()` resources clean karte hain.

**Subscriber Node (`simple_subscriber.py`):**
1.  **Imports:** Publisher ki tarah hi necessary libraries import kiye gaye hain.
2.  **Class `SimpleSubscriber`:** `Node` class se inherit karta hai.
    *   `__init__` constructor mein, `super().__init__('simple_subscriber')` se node initialize hota hai.
    *   `create_subscription` method ka use karke ek subscriber banaya gaya hai. Yeh `String` type ke messages `my_topic` se receive karega, aur jab bhi koi message receive hoga, `listener_callback` method execute hoga.
3.  **`listener_callback`:** Jab `my_topic` par koi message receive hota hai, toh yeh method us message ke data ko console par print karta hai.
4.  **`main` function:** Publisher ki tarah hi ROS2 initialization, node object creation, `rclpy.spin()`, aur resource cleanup operations involve hain.

## 6. Roman Urdu Comments

Upar diye gaye code examples mein har line ke saath Roman Urdu mein comments shamil kiye gaye hain taake non-technical background waley ya Urdu speakers ke liye code ko samajhna aasan ho. Yeh comments har line ke maqsad ko explain karte hain aur code ki functionality ko wazeh karte hain. Misaal ke taur par:

*   `import rclpy # rclpy library ko import kar rahe hain. Yeh ROS2 Python clients ke liye hai.`
    *   Yeh comment batata hai ke `rclpy` library ko kyun import kiya gaya hai.
*   `self.publisher_ = self.create_publisher(String, 'my_topic', 10) # Ek publisher bana rahe hain.`
    *   Yeh comment explain karta hai ke `create_publisher` method kya kar raha hai aur kis maqsad ke liye use ho raha hai.

Yeh tareeqa code ko more accessible banata hai aur learning process ko behtar karta hai.

## 7. MCQs (Multiple Choice Questions)

1.  **ROS2 mein data exchange ke liye nodes kis mechanism ka istemal karte hain?**
    a) Services
    b) Actions
    c) Topics
    d) Parameters

    **Correct Answer:** c) Topics

2.  **`rclpy` kis programming language ke liye ROS2 client library hai?**
    a) C++
    b) Java
    c) Python
    d) JavaScript

    **Correct Answer:** c) Python

3.  **Agar aapko do nodes ke darmiyan synchronous request/response communication chahiye, toh aap kya use karenge?**
    a) Topics
    b) Services
    c) Actions
    d) Messages

    **Correct Answer:** b) Services

4.  **ROS2 mein `rclpy.spin()` function ka kya maqsad hai?**
    a) ROS2 environment ko initialize karna
    b) Node ke callbacks ko process karna
    c) Message publish karna
    d) Node ko destroy karna

    **Correct Answer:** b) Node ke callbacks ko process karna

5.  **DDS (Data Distribution Service) ROS2 mein kis layer par kaam karta hai?**
    a) Application layer
    b) Transport layer
    c) Presentation layer
    d) Session layer

    **Correct Answer:** b) Transport layer

## 8. Troubleshooting

ROS2 develop karte waqt kuch common issues aa sakte hain:

*   **"Command not found" for `ros2`:** Iska matlab hai ke ROS2 environment source nahi kiya gaya hai. Har naye terminal mein `source /opt/ros/humble/setup.bash` (ya jo bhi aapki distribution hai) chalana zaroori hai. Ya isse `.bashrc` mein add karein.
*   **Nodes ek doosre ko nahi dekh pa rahe:** Ensure karein ke sabhi nodes ek hi ROS2 domain ID par chal rahe hain (`ROS_DOMAIN_ID` environment variable). Default 0 hota hai.
*   **Package nahi mil raha:** Agar aapne apna custom package banaya hai, toh use compile karne ke baad `source install/setup.bash` (workspace ke andar) karna zaroori hai taake system use detect kar sake.
*   **Compile errors:** CMakeLists.txt aur package.xml files ko sahi tarah se configure karein, aur ensure karein ke saari dependencies install hain.

## 9. Summary

Is introduction mein humne ROS2 ke bunyadi concepts, iski setup process, aur `rclpy` ka use karte hue simple publisher aur subscriber nodes ke code examples ko dekha. Humne Roman Urdu comments ke zariye code ko mazeed accessible banaya aur ROS2 se mutalliq kuch MCQs bhi discuss kiye. ROS2 modern robotics applications develop karne ke liye ek powerful aur flexible framework provide karta hai, jiski madad se complex systems ko modular tareeqe se banaya ja sakta hai.

## 10. Resources

*   **Official ROS2 Documentation:** [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/) (Replace `humble` with your ROS2 distribution)
*   **ROS2 Tutorials:** [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)
*   **ROS Community:** [https://answers.ros.org/](https://answers.ros.org/)
*   **`rclpy` API Reference:** [https://docs.ros.org/en/humble/p/rclpy/](https://docs.ros.org/en/humble/p/rclpy/)
