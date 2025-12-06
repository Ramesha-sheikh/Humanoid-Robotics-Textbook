# 03 - Building ROS 2 Packages

## Section 1: Introduction to ROS 2 Packages

ROS 2 (Robot Operating System 2) mein, packages software ki fundamental organizational units hotay hain. Har package ek specific functionality provide karta hai, jaisay device driver, set of algorithms, ya user-facing tools. Ye packages code ko modular aur reusable banate hain.

## Section 2: Creating a New ROS 2 Python Package

ROS 2 mein ek naya Python package create karna `ros2 pkg create` command se bohot easy hai. Hum `ament_python` build type use karte hain Python packages ke liye.

```bash
# Ek naya ROS 2 Python package banate hain jiska naam 'my_python_pkg' hai.
# 'ament_python' build type specify kar rahe hain Python packages ke liye.
ros2 pkg create --build-type ament_python my_python_pkg

```

Is command se `my_python_pkg` naam ka ek directory banega jismein basic structure aur files hongi, jaisay `package.xml` aur `setup.py`.

## Section 3: Understanding `setup.py`

`setup.py` Python packages ke liye standard build file hai. ROS 2 `ament_python` packages mein, yeh file `setuptools` use karta hai package ko define karne aur uski dependencies specify karne ke liye.

Ek typical `setup.py` file kuch aisa dikhega:

```python
from setuptools import setup

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_launch_file.launch.py']), # Example: Launch files add kar sakte hain
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name', # Apna naam yahan likhein
    maintainer_email='your_email@example.com', # Apna email yahan likhein
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_python_pkg.publisher_member_function:main', # Apna executable yahan define karein
            'listener = my_python_pkg.subscriber_member_function:main', # Doosra executable
        ],
    },
)


```

-   `package_name`: Aapke package ka naam.
-   `packages`: Python modules (directories) jo install kiye jayenge.
-   `data_files`: Non-Python files (jaisay `package.xml`, launch files, config files) jo install kiye jayenge.
-   `install_requires`: Is package ki Python dependencies.
-   `entry_points`: Console scripts define karne ke liye, jo aapke Python functions ko command line executables mein convert karte hain.

## Section 4: Understanding `package.xml`

`package.xml` ROS 2 packages ke liye manifest file hai. Ismein package ka metadata (jaisay naam, version, maintainer) aur uski dependencies (compile-time aur run-time) shamil hoti hain.

Ek typical `package.xml` file kuch aisa dikhega:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_python_pkg</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend> <!-- rclpy dependency -->
  <depend>std_msgs</depend> <!-- Standard messages dependency -->

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

```

-   `<name>`, `<version>`, `<description>`, `<maintainer>`, `<license>`: Package ki basic information.
-   `<depend>`: Dependencies jo run-time aur compile-time dono ke liye zaroori hain. `rclpy` aur `std_msgs` ROS 2 Python nodes ke liye common dependencies hain.
-   `<test_depend>`: Test dependencies.
-   `<export>`: Build system ko batata hai ke yeh `ament_python` type ka package hai.

## Section 5: `colcon` Build System

`colcon` ROS 2 ka primary build system hai. Yeh ek meta-build system hai jo multiple build systems (jaisay `ament`, `catkin`, `CMake`) ko support karta hai. `colcon` automatically dependencies ko detect karta hai aur packages ko sahi order mein build karta hai.

Common `colcon` commands:
-   `colcon build`: Workspace mein sab packages ko build karta hai.
-   `colcon test`: Build kiye gaye packages ke tests run karta hai.
-   `colcon clean`: Build files aur install directories ko remove karta hai.

## Section 6: Building a Python Package with `colcon`

Jab aapne apna Python package bana liya ho aur `setup.py` aur `package.xml` files configure kar liye hon, toh aap use `colcon` se build kar sakte hain.

Apne workspace root directory mein jaakar:

```bash
# Colcon build command, "--packages-select my_python_pkg" se sirf humara package build hoga.
# Agar ye option na ho, toh saare packages build honge workspace mein.
colcon build --packages-select my_python_pkg

```

Build successful hone par, aapke package ke executables aur libraries `install/my_python_pkg` directory mein milenge.

`colcon build` run karne ke baad, aapko environment ko source karna zaroori hai taake ROS 2 aapke naye package ko detect kar sake:

```bash
# Linux/macOS users ke liye
source install/setup.bash

# Windows users ke liye
call install/setup.bat

```

## Section 7: Writing a Simple `rclpy` Node (Publisher)

`rclpy` ROS 2 ke liye Python client library hai. Chaliye ek simple publisher node banate hain jo "Hello World" messages publish karega.

`my_python_pkg/my_python_pkg/publisher_member_function.py` mein yeh code likhein:

```python
import rclpy # ROS 2 Python client library import karte hain
from rclpy.node import Node # Node class ko import karte hain

from std_msgs.msg import String # String message type ko import karte hain


class MinimalPublisher(Node): # MinimalPublisher naam ki class banate hain jo Node se inherit karti hai

    def __init__(self):
        super().__init__('minimal_publisher') # Node ko 'minimal_publisher' naam dete hain
        # Publisher banate hain jo 'topic' par String messages publish karega, queue size 10 hai
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds # Timer period set karte hain
        # Timer banate hain jo har 0.5 second mein timer_callback function ko call karega
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0 # Counter initialize karte hain

    def timer_callback(self):
        msg = String() # Naya String message banate hain
        msg.data = 'Hello World: %d' % self.i # Message data set karte hain
        self.publisher_.publish(msg) # Message publish karte hain
        self.get_logger().info('Publishing: "%s"' % msg.data) # Console par publish kiya hua message print karte hain
        self.i += 1 # Counter increase karte hain


def main(args=None):
    rclpy.init(args=args) # rclpy ko initialize karte hain

    minimal_publisher = MinimalPublisher() # MinimalPublisher node create karte hain

    rclpy.spin(minimal_publisher) # Node ko spin karte hain taake callbacks process ho sakein

    # Destroy the node explicitly
    # aur resources release karein
    minimal_publisher.destroy_node()
    rclpy.shutdown() # rclpy ko shutdown karte hain


if __name__ == '__main__':
    main() # Main function call karte hain

```

## Section 8: Writing a Simple `rclpy` Node (Subscriber)

Ab ek simple subscriber node banate hain jo publisher se "Hello World" messages receive karega.

`my_python_pkg/my_python_pkg/subscriber_member_function.py` mein yeh code likhein:

```python
import rclpy # ROS 2 Python client library import karte hain
from rclpy.node import Node # Node class ko import karte hain

from std_msgs.msg import String # String message type ko import karte hain


class MinimalSubscriber(Node): # MinimalSubscriber naam ki class banate hain jo Node se inherit karti hai

    def __init__(self):
        super().__init__('minimal_subscriber') # Node ko 'minimal_subscriber' naam dete hain
        # Subscriber banate hain jo 'topic' se String messages receive karega, queue size 10 hai
        # Har message receive hone par listener_callback function ko call karega
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning # Warning se bachne ke liye

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data) # Console par receive kiya hua message print karte hain


def main(args=None):
    rclpy.init(args=args) # rclpy ko initialize karte hain

    minimal_subscriber = MinimalSubscriber() # MinimalSubscriber node create karte hain

    rclpy.spin(minimal_subscriber) # Node ko spin karte hain taake callbacks process ho sakein

    # Destroy the node explicitly
    # aur resources release karein
    minimal_subscriber.destroy_node()
    rclpy.shutdown() # rclpy ko shutdown karte hain


if __name__ == '__main__':
    main() # Main function call karte hain

```

**Important:** `setup.py` mein `entry_points` update karna na bhulein taake yeh nodes executables ban sakein. Sections 3 aur 7 mein diye gaye `setup.py` examples mein ye entry points already shamil hain.

## Section 9: Running the ROS 2 Nodes

Apne workspace root directory mein environment source karne ke baad, aap in nodes ko run kar sakte hain.

Pehle terminal mein publisher node run karein:

```bash
# Publisher node ko run karte hain
ros2 run my_python_pkg talker

```

Doosre terminal mein subscriber node run karein:

```bash
# Subscriber node ko run karte hain
ros2 run my_python_pkg listener

```

Aap dekhenge ke subscriber terminal par publisher ke messages receive ho rahe hain.

## Section 10: Summary and Conclusion

Is chapter mein humne ROS 2 Python packages banane, `setup.py` aur `package.xml` files ko configure karne, `colcon` build system ko use karne, aur simple `rclpy` publisher aur subscriber nodes banane ka tareeka seekha. ROS 2 mein modular aur maintainable robotics applications develop karne ke liye packages ki understanding bohot zaroori hai.

## Multiple Choice Questions (MCQs)

**1. ROS 2 mein Python packages banane ke liye kaunsa build type use hota hai?**
   a) `catkin`
   b) `cmake`
   c) `ament_python`
   d) `make`

**2. `setup.py` file mein `entry_points` ka kya maqsad hai?**
   a) Package ki dependencies define karna
   b) Non-Python files ko specify karna
   c) Python functions ko command line executables banana
   d) Package ka version specify karna

**3. `package.xml` file mein `<depend>` tag ka kya kaam hai?**
   a) Test dependencies specify karna
   b) Package ka description likhna
   c) Compile-time aur run-time dependencies declare karna
   d) Maintainer ka email specify karna

**4. ROS 2 ka primary build system kaunsa hai?**
   a) `catkin_make`
   b) `make`
   c) `colcon`
   d) `ament_cmake`

**5. `colcon build` command run karne ke baad, ROS 2 ko naye packages detect karne ke liye kya karna zaroori hai?**
   a) `ros2 run` command run karna
   b) `setup.py` file ko edit karna
   c) Environment ko source karna (`source install/setup.bash` ya `call install/setup.bat`)
   d) System ko restart karna
