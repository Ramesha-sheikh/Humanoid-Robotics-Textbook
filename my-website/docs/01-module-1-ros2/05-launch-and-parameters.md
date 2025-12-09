# ROS 2 Launch and Parameters

## Introduction
Is chapter mein, hum **ROS 2 Launch system** aur **Parameters** ke baare mein seekhenge. Launch files ka istemal multiple ROS 2 nodes ko aik saath start karne, unki configurations set karne, aur complex robot applications ko asani se manage karne ke liye hota hai. Hum Launch files ke bunyadi dhanchay aur unhein Python mein kaise likhte hain, is par tawajjuh denge. Hum yeh bhi dekhenge ke ROS 2 parameters kya hain aur unhein kaise nodes mein istemal kiya jata hai.

## Concepts
### ROS 2 Launch System
ROS 2 Launch system aik tool hai jo aapko ek XML ya Python file mein multiple nodes ko define karne, unke arguments set karne, aur unhein aik hi command se chalane ki ijazat deta hai. Iske bunyadi components hain:

*   **Launch Files**: Ye `.launch.py` (Python) ya `.launch.xml` (XML) files hoti hain jo ROS 2 application ki startup logic ko describe karti hain.
*   **Nodes**: Launch files mein aap nodes ko specify karte hain jinhein aap run karna chahte hain.
*   **Arguments (`Args`)**: Launch files mein aap arguments define kar sakte hain jinhein runtime par pass kiya ja sakta hai (jaise robot model ka path).
*   **Parameters (`Params`)**: Nodes ki runtime behavior ko configure karne ke liye parameters ka istemal hota hai.

### Why Use Launch Files?

*   **Automation**: Multiple nodes ko manually start karne ke bajaye, aik launch file sab ko aik saath chalata hai.
*   **Configuration**: Parameters aur arguments ke zariye nodes ki configuration asani se ki ja sakti hai.
*   **Reproducibility**: Development aur deployment environments mein application ki consistent behavior ko yaqeeni banata hai.
*   **Modularity**: Mukhtalif components ke liye alag alag launch files bana kar unhein compose kiya ja sakta hai.

### ROS 2 Parameters
ROS 2 parameters nodes ki configurable values hote hain jinhein runtime par modify kiya ja sakta hai. Ye key-value pairs ki shakal mein hote hain (jaise `robot_radius: 0.5`). Parameters ka istemal sensor configurations, controller gains, ya koi bhi numeric ya string value ko store karne ke liye hota hai jiski node ko zaroorat hoti hai.

*   **Types**: Parameters boolean, integer, double, string, aur lists of these types ho sakte hain.
*   **Access**: Nodes `rclpy` (Python) ya `rclcpp` (C++) client libraries ka istemal kar ke parameters ko declare, get, set aur modify kar sakte hain.
*   **Command Line**: Parameters ko `ros2 run` ya `ros2 launch` commands ke zariye bhi set kiya ja sakta hai.

## Setup
Launch files aur parameters ke saath kaam karne ke liye, aapko ROS 2 environment ki zaroorat hogi. Apne workspace mein ek naya package banayenge agar aapne abhi tak nahi banaya. Pichle chapters ke steps ko follow kar ke ek naya Python package create karein, jiska naam `my_launch_tutorial` ya koi bhi suitable naam ho sakta hai.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_launch_tutorial --dependencies rclpy
cd ~/ros2_ws
colcon build
source install/setup.bash
```
*Yeh commands ek naya Python package `my_launch_tutorial` banayenge aur usko build kar ke workspace ko source karenge.* (These commands will create a new Python package `my_launch_tutorial`, build it, and source the workspace.)

Ab hum apne `my_launch_tutorial` package mein launch files aur Python nodes banayenge.

## rclpy Launch File Examples

### 1. Simple Talker Node (Python)

Sab se pehle, hum ek simple Python node banayenge jo ROS 2 topic par messages publish karega. `~/ros2_ws/src/my_launch_tutorial/my_launch_tutorial/simple_talker.py` naam ki file banayein:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleTalker(Node):
    def __init__(self):
        super().__init__('simple_talker')
        self.publisher_ = self.create_publisher(String, 'chat_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from simple_talker: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Yeh ek bunyadi `rclpy` node hai jo `chat_topic` par har second "Hello from simple_talker" message publish karta hai.* (This is a basic `rclpy` node that publishes "Hello from simple_talker" message every second on `chat_topic`.)

`~/ros2_ws/src/my_launch_tutorial/setup.py` mein `entry_points` mein is node ko add karein:

```python
        'console_scripts': [
            'simple_talker = my_launch_tutorial.simple_talker:main',
        ],
```

Build karein aur source karein:
```bash
cd ~/ros2_ws
colcon build --packages-select my_launch_tutorial
source install/setup.bash
```

### 2. Basic Launch File (Python)

Ab hum is node ko launch file ke zariye chalayenge. `~/ros2_ws/src/my_launch_tutorial/launch/simple_launch.launch.py` naam ki file banayein:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_launch_tutorial',
            executable='simple_talker',
            name='my_simple_talker',
            output='screen',
            emulate_tty=True,  # Required for seeing print statements in the console
        )
    ])
```
*Yeh Python launch file `simple_talker` node ko start karta hai. `output='screen'` console par output dikhane ke liye hai.* (This Python launch file starts the `simple_talker` node. `output='screen'` is for showing output on the console.)

Launch karein:

```bash
ros2 launch my_launch_tutorial simple_launch.launch.py
```

### 3. Launch File with Parameters (Python)

Parameters nodes ki behavior ko runtime par modify karne ki ijazat dete hain. Sab se pehle, ek naya Python node banayenge jo parameter ka istemal karega. `~/ros2_ws/src/my_launch_tutorial/my_launch_tutorial/param_talker.py` naam ki file banayein:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ParamTalker(Node):
    def __init__(self):
        super().__init__('param_talker')
        self.declare_parameter('message_prefix', 'Default Message')
        self.declare_parameter('publish_frequency', 1.0)

        self.message_prefix = self.get_parameter('message_prefix').get_parameter_value().string_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(String, 'param_topic', 10)
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.message_prefix}: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = ParamTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Yeh node do parameters declare karta hai: `message_prefix` aur `publish_frequency`. Ye parameters node ki output aur publishing rate ko control karte hain.* (This node declares two parameters: `message_prefix` and `publish_frequency`. These parameters control the node's output and publishing rate.)

`~/ros2_ws/src/my_launch_tutorial/setup.py` mein `entry_points` mein is node ko bhi add karein:

```python
        'console_scripts': [
            'simple_talker = my_launch_tutorial.simple_talker:main',
            'param_talker = my_launch_tutorial.param_talker:main',
        ],
```

Build karein aur source karein:
```bash
cd ~/ros2_ws
colcon build --packages-select my_launch_tutorial
source install/setup.bash
```

Ab, ek launch file banayenge jo in parameters ko set karega. `~/ros2_ws/src/my_launch_tutorial/launch/param_launch.launch.py` naam ki file banayein:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    message_prefix_arg = DeclareLaunchArgument(
        'message_prefix',
        default_value=TextSubstitution(text='Custom Prefix')
    )
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value=TextSubstitution(text='0.5')
    )

    return LaunchDescription([
        message_prefix_arg,
        publish_frequency_arg,
        Node(
            package='my_launch_tutorial',
            executable='param_talker',
            name='my_param_talker',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'message_prefix': LaunchConfiguration('message_prefix')
                },
                {
                    'publish_frequency': LaunchConfiguration('publish_frequency')
                }
            ]
        )
    ])
```
*Yeh launch file `param_talker` node ko chalata hai aur `message_prefix` aur `publish_frequency` parameters ko `Custom Prefix` aur `0.5` par set karta hai. Aap in values ko command line se override bhi kar sakte hain.* (This launch file runs the `param_talker` node and sets the `message_prefix` and `publish_frequency` parameters to `Custom Prefix` and `0.5`. You can also override these values from the command line.)

Launch karein, default parameters ke saath:

```bash
ros2 launch my_launch_tutorial param_launch.launch.py
```

Parameters ko override karte hue launch karein:

```bash
ros2 launch my_launch_tutorial param_launch.launch.py message_prefix:="Overridden Message" publish_frequency:=2.0
```
*Yeh command parameters ko override kar dega, aur node "Overridden Message" har 0.5 seconds mein publish karega.* (This command will override the parameters, and the node will publish "Overridden Message" every 0.5 seconds.)

## Explanation

Is section mein, humne **ROS 2 Launch system** aur **Parameters** ka istemal seekha. Launch files multiple nodes ko organize aur start karne ke liye aik zaroori tareeqa hain. Humne dekha ke kaise Python mein launch files banaye jate hain aur unmein nodes aur parameters ko kaise define kiya jata hai. Parameters nodes ki dynamic configuration allow karte hain, jis se unki behavior ko runtime par modify kiya ja sakta hai.

### Key Takeaways:

*   **Launch File Structure**: Python launch files `LaunchDescription` object return karte hain jismein `Node` actions shamil hote hain.
*   **Node Configuration**: Launch files mein `package`, `executable`, `name`, aur `output` jaise attributes se nodes ko configure kiya jata hai.
*   **Parameters in Nodes**: Nodes `declare_parameter`, `get_parameter`, aur `set_parameter` methods ka istemal kar ke parameters se interact karte hain.
*   **Parameters in Launch Files**: `DeclareLaunchArgument` se launch arguments define kiye jate hain, aur `LaunchConfiguration` se unhein nodes ke parameters mein pass kiya jata hai.
*   **Overriding Parameters**: Parameters ko command line se bhi override kiya ja sakta hai, jis se debugging aur testing asan ho jata hai.

Yeh samajh aapko complex ROS 2 applications ki deployment aur configuration mein madad karegi.

## Multiple Choice Questions (MCQs)

1.  ROS 2 Launch system ka bunyadi maqsad kya hai?
    a) Single node ko run karna
    b) Multiple nodes ko aik saath start aur configure karna
    c) ROS 2 packages ko build karna
    d) ROS 2 topics ko list karna
    **Correct Answer: b) Multiple nodes ko aik saath start aur configure karna**

2.  Python launch files mein `LaunchDescription` object kya return karta hai?
    a) Aik string
    b) Aik integer
    c) Actions ki list (jaise `Node`)
    d) Aik dictionary
    **Correct Answer: c) Actions ki list (jaise `Node`)**

3.  ROS 2 parameters kis liye istemal hote hain?
    a) Nodes ke names define karne ke liye
    b) Nodes ki runtime behavior ko configure karne ke liye
    c) ROS 2 messages ko encrypt karne ke liye
    d) ROS 2 graph ko visualize karne ke liye
    **Correct Answer: b) Nodes ki runtime behavior ko configure karne ke liye**

4.  Aik launch file mein parameters ko kaise pass kiya jata hai?
    a) `Node` action mein `arguments` attribute se
    b) `Node` action mein `parameters` attribute se `LaunchConfiguration` ka istemal karte hue
    c) `Node` action mein `data` attribute se
    d) Directly XML tags ka istemal karte hue
    **Correct Answer: b) `Node` action mein `parameters` attribute se `LaunchConfiguration` ka istemal karte hue**

5.  Parameters ko command line se override karne ka kya faida hai?
    a) Launch file ko modify kiye bagair node ki behavior badalna
    b) ROS 2 network traffic ko kam karna
    c) Node ki performance ko optimize karna
    d) Sirf XML launch files mein kaam karta hai
    **Correct Answer: a) Launch file ko modify kiye bagair node ki behavior badalna**

## Troubleshooting

*   **Launch file nahi chal raha**: Launch file ka syntax check karein (Python ke rules), aur make sure karein ke `setup.py` mein `data_files` mein launch files shamil kiye gaye hain taake `ros2 launch` unhein dhoond sake.
*   **Node launch hone ke baad band ho jata hai**: Node ke code mein errors check karein (jaise import errors, runtime exceptions). `output='screen'` aur `emulate_tty=True` ko `Node` action mein set karein taake console output dekh sakein.
*   **Parameters apply nahi ho rahe**: Make sure karein ke node `declare_parameter` ka istemal kar ke parameter ko declare kar raha hai aur launch file mein `parameters` attribute theek se set kiya gaya hai. Command line se parameters override karte waqt syntax (`param_name:=value`) ka khayal rakhein.

## Summary

Is chapter mein, humne ROS 2 Launch system aur parameters ki ahmiyat ko samjha, jo complex robot applications ki deployment aur dynamic configuration ke liye zaroori hain. Humne Python mein launch files banane, multiple nodes ko aik saath chalane, aur parameters ka istemal kar ke nodes ki behavior ko runtime par modify karne ka tareeqa seekha. Yeh skills aapko bade aur ziada flexible ROS 2 projects banane mein madad karegi.

## Resources

*   [ROS 2 Documentation: Launching nodes](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
*   [ROS 2 Documentation: Using parameters in a launch file](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Parameters-With-Launch-Files.html)
*   [ROS 2 Documentation: Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Understanding-ROS2-Parameters.html)


