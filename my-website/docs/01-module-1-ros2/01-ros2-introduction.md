# 01 - ROS 2 Introduction

## 1. Introduction
Is section mein hum ROS 2 (Robot Operating System 2) ka taaruf pesh karenge. Yeh ek open-source framework hai jo robotic applications ki development mein madad karta hai. Hum ROS1 par iski behtar karkardagi par bhi baat karenge, khaas taur par real-time performance, multi-robot systems, aur embedded platforms ke hawalay se.

## 2. Concepts
Is section mein ROS 2 ke bunyadi tasawwur tafseel se bayan kiye gaye hain, jin mein shaamil hain:
*   **Nodes:** Aise executable processes jo khaas kaam anjaam dete hain.
*   **Topics:** Data ke tabadlay ke liye ek se zyada nodes ke darmyan communication ka zariya.
*   **Messages:** Topics par transmit kiye jane wale data structures.
*   **Services:** Synchronous request/response communication ke liye istemal hote hain.
*   **Actions:** Lambay arsay tak chalne wale tasks ke liye, jin mein feedback aur cancellation ki sahulat hoti hai.
*   **ROS Graph:** Tamam mutasira nodes, topics, services, aur actions ka majmooa.
*   **DDS (Data Distribution Service):** Data ki munasib aur asardar delivery ke liye underlying transport layer.

## 3. Setup
ROS 2 ko Ubuntu jaisi Linux distribution par set up karne ke liye aala satah ki hidayat di gayi hain. Is mein repository setup, ROS 2 desktop packages install karna, aur setup script ko source karna shaamil hai, jis mein sourcing ko mustaqil banane ke liye notes bhi hain.

## 4. Code Examples
Is ahem section mein aik simple publisher aur subscriber node ke liye runnable `rclpy` (Python) code examples shaamil hain.

### Publisher Node (`simple_publisher.py`)
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher_node')
        # Publisher banayen jo 'my_topic' par String messages publish karega
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        # Har 0.5 seconds par message publish karne ke liye timer banayen
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2 from Publisher: {self.i}' # Message data tayyar karen
        self.publisher_.publish(msg) # Message publish karen
        self.get_logger().info(f'Publishing: "{msg.data}"') # Console par publish message dikhayen
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher) # Node ko chalayen taake callbacks execute hon
    simple_publisher.destroy_node() # Node ko khatam karen
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node (`simple_subscriber.py`)
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber_node')
        # Subscriber banayen jo 'my_topic' se String messages receive karega
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        # Receive shuda message ko console par dikhayen
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber) # Node ko chalayen
    simple_subscriber.destroy_node() # Node ko khatam karen
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. Explanation
Is section mein publisher aur subscriber code examples ki tafseeli tashreeh di gai hai. Har import, class method (`__init__`, `create_publisher`, `create_timer`, `timer_callback`, `create_subscription`, `listener_callback`), aur `main` function ke کردار ko bayan kiya gaya hai jo ROS 2 components ko initialize, spin, aur shut down karne mein madad karta hai.

## 6. Roman Urdu Comments
Is section mein code examples mein Roman Urdu comments ki ahmiyat aur maqsad ko bayan kiya gaya hai. Yeh comments Urdu bolne walon ya ghair takneeki afraad ke liye code ki har line ke function ko wazeh kar ke samjhne mein madad karte hain, is terha accessibility aur learning process behtar hoti hai.

## 7. MCQs (Multiple Choice Questions)
ROS 2 ke bunyadi tasawwur se mutaliq paanch multiple-choice questions shamil kiye gaye hain, jin ke durust jawabat bhi diye gaye hain. Yeh questions data exchange mechanisms, client libraries, lambay arsay tak chalne wale tasks ke liye communication patterns, build systems, aur API usage jaise topics par mabni hain.

1.  **ROS 2 mein data ke tabadlay ka bunyadi mechanism kya hai?**
    a) Services
    b) Actions
    c) Topics
    d) Parameters
    **Correct Answer:** c) Topics

2.  **ROS 2 mein Python client library ka naam kya hai?**
    a) rospy
    b) roscpp
    c) rclpy
    d) rosjava
    **Correct Answer:** c) rclpy

3.  **Lambay arsay tak chalne wale tasks jin mein feedback aur cancellation ki zaroorat ho, un ke liye kaunsa ROS 2 communication pattern istemal hota hai?**
    a) Topics
    b) Services
    c) Actions
    d) Parameters
    **Correct Answer:** c) Actions

4.  **ROS 2 packages ko build karne ke liye kaunsa build system istemal hota hai?**
    a) catkin
    b) make
    c) colcon
    d) cmake
    **Correct Answer:** c) colcon

5.  **`rclpy.spin()` function ka bunyadi maqsad kya hai?**
    a) ROS 2 node ko initialize karna
    b) Node ke callbacks ko execute karna
    c) ROS 2 network ko shut down karna
    d) Message publish karna
    **Correct Answer:** b) Node ke callbacks ko execute karna

## 8. Troubleshooting
ROS 2 development ke dauran aane wale aam masail aur un ke hal is section mein bayan kiye gaye hain:

*   **"Command not found" for `ros2`:** Yaqeen banayen ke aap ne ROS 2 setup script ko source kiya hai: `source /opt/ros/humble/setup.bash`.
*   **Nodes communicate nahi kar rahe hain:** Topic names, message types, aur QoS settings ko verify karen. Ensure ke dono nodes aik hi ROS domain ID par chal rahe hain.
*   **Packages nahi mil rahe hain:** Yaqeen banayen ke aap ne `colcon build` kiya hai aur apne workspace ko source kiya hai: `source install/setup.bash`.
*   **Compilation errors:** Dependencies ko double-check karen aur `colcon build --symlink-install` ya `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` istemal karne ki koshish karen.

## 9. Summary
Yeh document ROS 2 ka ek mukhtasar khulasa pesh karta hai, jis mein iski bunyadi salahiyaton ko wazeh kiya gaya hai. ROS 2 aik taqatwar aur flexible framework hai jo modular robotics application development ke liye behtareen hai.

## 10. Resources
mazeed maloomat ke liye chand ahem resources ki fehrist:

*   **Official ROS 2 Documentation:** [https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/index.html)
*   **ROS 2 Tutorials:** [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)
*   **ROS Community Forum:** [https://answers.ros.org/](https://answers.ros.org/)
*   **rclpy API Reference:** [https://docs.ros.org/en/humble/p/rclpy/](https://docs.ros.org/en/humble/p/rclpy/)
