# Building Your First ROS 2 Package in Python

## Introduction
Welcome to the exciting world of ROS 2 package development! In this chapter, we will guide you through the process of creating your very first ROS 2 package using Python. Packages are the fundamental building blocks in ROS 2, encapsulating code, data, dependencies, and configurations. Understanding how to create and manage them is crucial for any robotics project. By the end of this chapter, you will have a working ROS 2 Python package and a solid understanding of its basic structure.

## Concepts
Before we dive into creating a package, let's understand some core concepts:

### ROS 2 Package Structure
A ROS 2 package is essentially a directory with a specific layout that allows ROS 2 to find and use its contents. Key files include:
*   `package.xml`: This file contains metadata about your package, such as its name, version, description, maintainers, license, and most importantly, its dependencies.
    *   *Yeh aapke package ki identity card hai, saari zaroori details ismein hoti hain.* (This is your package's identity card, all essential details are in it.)
*   `setup.py` (for Python packages): This Python script is used by `colcon` to build and install your Python code. It specifies entry points (your executable scripts), dependencies, and other Python-specific configurations.
    *   *Python code ko install aur build karne ke liye yeh script zaroori hai.* (This script is necessary for installing and building Python code.)
*   `resource/` folder: Contains marker files (e.g., `ament_python`) that tell ROS 2 the type of your package.

### Build System: Colcon
`colcon` is the primary build tool for ROS 2. It orchestrates the building, testing, and installing of multiple packages in a workspace.
*   *Colcon ek tarah ka manager hai jo aapke saare packages ko theek se banane mein madad karta hai.* (Colcon is like a manager that helps build all your packages properly.)

### Workspace
A ROS 2 workspace is a directory where you develop, build, and install your packages. It typically contains:
*   `src/`: Where your package source code resides.
*   `install/`: Where built packages are installed.
*   `log/`: Build logs.
*   `build/`: Intermediate build files.
    *   *Yeh aapki coding ki dukan hai jahan saare packages banaye aur rakhe jaate hain.* (This is your coding workshop where all packages are built and kept.)

### Nodes
Within a ROS 2 package, the main computational units are called \"nodes\". A node is essentially an executable program that performs a specific task. For Python packages, these nodes are often Python scripts.

## Setup
To begin, you'll need a ROS 2 environment set up (as covered in previous chapters). Assuming that's ready, let's create a dedicated workspace and then our first package.

### 1. Create a ROS 2 Workspace
It's good practice to create a new workspace for your development.

```
bash
# Create a new directory for your workspace
mkdir -p ~/ros2_ws/src
# Navigate into the workspace directory
cd ~/ros2_ws

```

### 2. Source Your ROS 2 Environment
Ensure your ROS 2 environment is sourced in every new terminal you open. This makes ROS 2 commands available.

```
bash
# Replace 'humble' with your ROS 2 distribution name if different (e.g., 'foxy', 'galactic')
source /opt/ros/humble/setup.bash
# If you have an overlay workspace, source its setup file after the main ROS 2 setup
# source ~/ros2_ws/install/setup.bash

```

### 3. Create Your First ROS 2 Python Package
Now, let's use the `ros2 pkg create` command to generate the basic structure of a Python package.

```
bash
# Navigate to the src directory of your workspace
cd ~/ros2_ws/src

# Create a new package named 'my_py_pkg' with Python as the build type
ros2 pkg create --build-type ament_python my_py_pkg

```

This command will create a new directory `my_py_pkg` inside your `src` folder, pre-populated with essential files like `package.xml`, `setup.py`, and `my_py_pkg/` (a subdirectory for your Python modules).

You have now successfully set up your environment and created the basic structure for your first ROS 2 Python package!

## Code Examples

Let's create a simple Python node within our `my_py_pkg` and then build and run it.

### 1. Create a Simple Talker Node

Inside your `~/ros2_ws/src/my_py_pkg/my_py_pkg` directory (you might need to create this inner directory if `ros2 pkg create` didn't), create a file named `talker.py` with the following content:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    talker_node = Talker()

    rclpy.spin(talker_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    talker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
*Yeh code ek simple ROS 2 publisher node banata hai jo har 0.5 seconds mein "Hello ROS 2" message publish karta hai.* (This code creates a simple ROS 2 publisher node that publishes "Hello ROS 2" message every 0.5 seconds.)

### 2. Update `setup.py`

To make your Python script an executable entry point, you need to modify your `setup.py` file. Open `~/ros2_ws/src/my_py_pkg/setup.py` and add the following lines to the `entry_points` dictionary:

```python
        'console_scripts': [
            'talker = my_py_pkg.talker:main',
        ],

```
Your `setup.py` should look something like this (ensure proper indentation):

```python
from setuptools import find_packages, setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A minimal ROS 2 Python package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_py_pkg.talker:main',
        ],
    },
)
```
*Humne `setup.py` mein `talker` node ko register kiya hai taake `ros2 run` command isse execute kar sake.* (We have registered the `talker` node in `setup.py` so that the `ros2 run` command can execute it.)

### 3. Build Your Package

Now, navigate back to your workspace root and build your package using `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select my_py_pkg
```
*Yeh command sirf `my_py_pkg` ko build karega. Agar koi error na ho, toh build successful ho jayega.* (This command will only build `my_py_pkg`. If there are no errors, the build will be successful.)

### 4. Source the Workspace

After building, you need to source your workspace to make the newly built executables available to your environment:

```bash
cd ~/ros2_ws
source install/setup.bash
```
*Build ke baad, naye executables ko istemal karne ke liye workspace ko source karna zaroori hai.* (After building, it's necessary to source the workspace to use the new executables.)

### 5. Run Your Node

Finally, you can run your `talker` node:

```bash
ros2 run my_py_pkg talker
```
*Yeh command `talker` node ko chalayega, aur aap console par "Publishing: Hello ROS 2: X" messages dekhenge.* (This command will run the `talker` node, and you will see "Publishing: Hello ROS 2: X" messages on the console.)

To see the messages being published, open a new terminal, source your ROS 2 environment and workspace, and then run:

```bash
ros2 topic echo /topic
```
*Yeh command `/topic` par publish hone wale messages ko display karega.* (This command will display the messages being published on `/topic`.)

## Explanation

Is section mein, humne dekha ke kaise ek **ROS 2 Python package** banaya jata hai aur uske andar ek simple publisher node ko implement, build aur execute kiya jata hai. Package ki structure, `package.xml`, `setup.py` ki ahmiyat, aur `colcon` build system ka istemal samjhaya gaya. `rclpy` library ka istemal karte hue, humne ek `Node` banaya jo `String` type ke messages ko `/topic` par publish karta hai.

### Key Takeaways:

*   **Package Creation**: `ros2 pkg create --build-type ament_python <package_name>` command se basic package structure generate hoti hai.
*   **Node Implementation**: `rclpy.node.Node` class ko extend kar ke nodes banaye jate hain. `create_publisher` aur `create_timer` methods ka istemal message publish karne ke liye hota hai.
*   **`setup.py` Configuration**: Python nodes ko executable banane ke liye `entry_points` mein `console_scripts` define karna zaroori hai.
*   **Building with `colcon`**: `colcon build` command packages ko compile aur install karta hai. `--packages-select` option se sirf specific package ko build kiya ja sakta hai.
*   **Sourcing Workspace**: Build ke baad, naye executables ko environment mein shamil karne ke liye `source install/setup.bash` zaroori hai.
*   **Running Nodes**: `ros2 run <package_name> <executable_name>` se node ko chalaya jata hai.
*   **Topic Monitoring**: `ros2 topic echo <topic_name>` se kisi bhi topic par publish hone wale messages ko monitor kiya ja sakta hai.

Yeh bunyadi steps aapko ROS 2 mein apne khud ke robotics applications banane mein madad karenge.

## Multiple Choice Questions (MCQs)

1.  ROS 2 mein code ko organize karne ka bunyadi tareeqa kya hai?
    a) Nodes
    b) Topics
    c) Packages
    d) Services
    **Correct Answer: c) Packages**

2.  `package.xml` file mein kya information hoti hai?
    a) Sirf package ka naam
    b) Package ka naam, version, maintainer aur dependencies
    c) Node ke functions
    d) Build commands
    **Correct Answer: b) Package ka naam, version, maintainer aur dependencies**

3.  ROS 2 mein default build system kaunsa hai?
    a) Catkin
    b) Make
    c) colcon
    d) CMake
    **Correct Answer: c) colcon**

4.  Python ROS 2 package mein `setup.py` ka kya maqsad hai?
    a) Package ki dependencies define karna
    b) Python code ko build aur install karna
    c) ROS topics ko create karna
    d) Nodes ko run karna
    **Correct Answer: b) Python code ko build aur install karna**

5.  `ros2 run <package_name> <executable_name>` command kya karta hai?
    a) Naya package banata hai
    b) Package ko compile karta hai
    c) Ek node ko execute karta hai
    d) ROS topics ko list karta hai
    **Correct Answer: c) Ek node ko execute karta hai**

## Troubleshooting

*   **Package not found**: Agar `ros2 run` ya `colcon build` ke dauran package nahi milta, toh ya toh aap workspace ko source karna bhool gaye hain, ya `setup.py` mein entry point theek se configure nahi kiya gaya.
*   **Build errors**: Dependencies check karein (`package.xml`), syntax errors (`setup.py` aur Python files) aur make sure karein ke sab theek ho.
*   **Node not publishing/subscribing**: `ros2 topic list`, `ros2 node list` aur `ros2 topic echo` ka istemal kar ke topics aur nodes ki connectivity check karein.

## Summary

Is chapter mein, humne ROS 2 packages banane, unhein `colcon` se build karne, aur basic `rclpy` nodes ko run karne ke bunyadi tareeqon ko seekha. Packages modular development ke liye zaroori hain, aur `colcon` ek efficient build system hai. Is knowledge ke saath, aap apne khud ke ROS 2 applications develop karna shuru kar sakte hain.

## Resources

*   [ROS 2 Documentation: Creating a package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
*   [ROS 2 Documentation: Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Publisher-And-Subscriber-Py.html)
*   [colcon Documentation](https://colcon.readthedocs.io/en/main/)


