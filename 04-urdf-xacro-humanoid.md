# 04 - URDF aur Xacro ka Istemal: Humanoid Robots ki Tafseel

Is chapter mein, hum URDF (Unified Robot Description Format) aur Xacro (XML Macros) ke gehre mutaleh mein jayenge, jo ke humanoid robots ko bayan karne ke liye behtareen tools hain. Hum seekhenge ke kaise in tools ka istemal karte hue hum robots ke physical aur kinematic properties ko define kar sakte hain.

## 1. Robot Tafseel ka Taaruf (Introduction to Robot Description)

Robots ko control karne, simulate karne, aur visualize karne ke liye, unki physical structure aur kinematic properties ko bayan karna zaroori hai. URDF aur Xacro is maqsad ke liye istemal hone wale ahem formats hain. URDF robots ke links aur joints ki aik standard XML-based tafseel faraham karta hai, jabke Xacro URDF files ko mazeed modular aur parameterizable banata hai.

## 2. URDF (Unified Robot Description Format) ko Samjhna

URDF aik XML format hai jo robot ke tamam links (physical parts) aur joints (links ko jodne wale hisse) ko bayan karta hai.
URDF ke ahem tags:
- `<robot>`: Root element.
- `<link>`: Robot ka aik physical part. Is mein visual (shakal), collision (takrao), aur inertial (wazan, mass distribution) properties hoti hain.
- `<joint>`: Do links ko jorne wala mechanism. Is mein type (revolute, prismatic, fixed, continuous, planar, floating), parent aur child links, origin (position aur orientation), aur axis (ghoomne ya move karne ki simt) hote hain.

**Link Properties:**
- `<visual>`: Robot ki visual appearance (mesh file, material, origin).
- `<collision>`: Robot ke collision geometry (mesh file, origin).
- `<inertial>`: Link ka mass, center of mass, aur inertia tensor.

**Joint Properties:**
- `type`: Joint ki qism (e.g., `revolute` ghoomne wala, `fixed` hamesha ke liye jura hua).
- `origin`: Parent link ke reference frame mein joint ki position aur orientation.
- `parent`, `child`: Joint kin do links ko jorta hai.
- `axis`: Ghoomne ya slide karne ka axis.

## 3. Aik Sadah Link ke Liye Bunyadi URDF Misaal

Yeh aik cylinder link ki sadah URDF tafseel hai:

```xml
<?xml version="1.0"?>
<robot name="simple_cylinder">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
</robot>
```

## 4. Xacro (XML Macros) ka Taaruf

URDF files, khas tor par complex robots ke liye, bahut lambi aur dohrane wali ho sakti hain. Xacro (XML Macros) is masle ko hal karta hai. Yeh XML preprocessor hai jo URDF files mein macros, properties, aur conditionals istemal karne ki ijazat deta hai. Is se code reusable, modular aur asani se parameterizable ho jata hai.

**Xacro ke Ahem Features:**
- `xacro:macro`: Reusable code blocks banane ke liye.
- `xacro:property`: Values ko define karne aur unhe baad mein istemal karne ke liye.
- `xacro:include`: Doosri Xacro files ko shamil karne ke liye.
- `$(arg name)`: Arguments ko pass karne ke liye.
- `$(op expression)`: Basic math operations karne ke liye.

## 5. Humanoid Robot Components ke Liye Xacro

Aik humanoid robot ke aksar hisse (jaise legs, arms) similar hote hain. Xacro ka istemal karte hue, hum in components ke liye macros bana sakte hain.

**Misaal: Aik Sadah Leg Component**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:property name="leg_mass" value="1.0" />
  <xacro:property name="leg_radius" value="0.05" />
  <xacro:property name="leg_length" value="0.5" />

  <xacro:macro name="leg_section" params="prefix offset_x offset_y offset_z">
    <link name="${prefix}_link">
      <visual>
        <geometry>
          <cylinder length="${leg_length}" radius="${leg_radius}"/>
        </geometry>
        <material name="green">
          <color rgba="0 0.8 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${leg_length}" radius="${leg_radius}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${leg_mass}"/>
        <inertia ixx="${(leg_mass / 12) * (3 * leg_radius*leg_radius + leg_length*leg_length)}" ixy="0.0" ixz="0.0"
                 iyy="${(leg_mass / 12) * (3 * leg_radius*leg_radius + leg_length*leg_length)}" iyz="0.0" izz="${(leg_mass / 2) * leg_radius*leg_radius}"/>
      </inertial>
    </link>

    <joint name="${prefix}_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_link"/>
      <origin xyz="${offset_x} ${offset_y} ${offset_z}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="100"/>
    </joint>
  </xacro:macro>

</robot>
```

## 6. Xacro ka Istemal Karte Hue Humanoid Robot Banana

Ab hum pichli section mein banaye gaye `leg_section` macro ka istemal karte hue aik simple humanoid robot assemble kar sakte hain.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Include our leg section macro -->
  <xacro:include filename="$(find your_package_name)/urdf/leg_macro.xacro" />

  <!-- Base Link of the Humanoid -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <xacro:leg_section prefix="left_hip" offset_x="0.0" offset_y="0.1" offset_z="-0.2"/>
  <xacro:leg_section prefix="left_knee" offset_x="0.0" offset_y="0.1" offset_z="-0.7"/>

  <!-- Right Leg -->
  <xacro:leg_section prefix="right_hip" offset_x="0.0" offset_y="-0.1" offset_z="-0.2"/>
  <xacro:leg_section prefix="right_knee" offset_x="0.0" offset_y="-0.1" offset_z="-0.7"/>

  <!-- You would add arms, torso, head similarly using more macros -->

</robot>
```
**Note:** `your_package_name` ko apne ROS 2 package ke naam se badlen. `leg_macro.xacro` file ko `urdf` folder mein rakhen.

## 7. `rclpy` ka Istemal Karte Hue URDF Parse Karna aur Robot State Publish Karna

`rclpy` ke zariye, hum URDF files ko load kar sakte hain, robot ke structure ko parse kar sakte hain, aur uski current joint positions ko `/joint_states` topic par publish kar sakte hain taake RViz jaisi tools usay visualize kar saken.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import os
import xacro
from urdf_parser_py.urdf import URDF # pip install urdf_parser_py

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher_node')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states) # Har 0.1 second mein publish karega
        self.joint_positions = {}
        self.joint_velocity = 0.1 # Joint ki raftaar
        self.joint_names = [] # Saare joints ke naam yahan honge

        # URDF/Xacro file load karein
        try:
            # Xacro file ka path
            xacro_file = os.path.join(
                os.path.dirname(__file__), '..', 'urdf', 'humanoid_robot.xacro'
            )

            # Agar Xacro hai, toh usay URDF mein expand karein
            if xacro_file.endswith('.xacro'):
                robot_description_raw = xacro.process_file(xacro_file).toprettyxml(indent='  ')
            else:
                with open(xacro_file, 'r') as file:
                    robot_description_raw = file.read()

            # URDF parse karein
            self.robot_model = URDF.from_xml_string(robot_description_raw)
            self.get_logger().info('URDF/Xacro file successfully loaded and parsed.')

            # Joints ke naam jama karein
            for joint in self.robot_model.joints:
                if joint.joint_type != 'fixed': # Fixed joints ko publish karne ki zaroorat nahi hoti
                    self.joint_names.append(joint.name)
                    self.joint_positions[joint.name] = 0.0 # Initial position

            self.get_logger().info(f"Detected joints: {self.joint_names}")

        except Exception as e:
            self.get_logger().error(f"Failed to load or parse URDF/Xacro file: {e}")
            rclpy.shutdown() # Agar file load na ho paye toh node ko band kar dein

    def publish_joint_states(self):
        # Jab file load na hui ho toh skip karein
        if not hasattr(self, 'robot_model'):
            return

        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        joint_state_msg.name = self.joint_names

        # Joints ki positions ko update karein (sirf misaal ke taur par)
        positions = []
        for joint_name in self.joint_names:
            # Aik simple oscillating movement
            self.joint_positions[joint_name] = math.sin(self.get_clock().now().nanoseconds / 1e9 * self.joint_velocity) * (math.pi / 4)
            positions.append(self.joint_positions[joint_name])

        joint_state_msg.position = positions
        joint_state_msg.velocity = [0.0] * len(self.joint_names) # Velocity ko 0 set karein
        joint_state_msg.effort = [] # Effort ko khali chhor dein

        self.publisher_.publish(joint_state_msg)
        # self.get_logger().info('Joint states published') # Har baar publish hone par log message na dein

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node) # Node ko chalate rahen
    node.destroy_node() # Node ko khatam karein
    rclpy.shutdown() # rclpy ko band karein

if __name__ == '__main__':
    main()
```
**`setup.py` mein entry point add karein:**
Apne ROS 2 package ke `setup.py` file mein, `entry_points` section mein yeh add karein:
```python
    entry_points={
        'console_scripts': [
            'robot_state_publisher = your_package_name.robot_state_publisher_node:main',
        ],
    },
```
**Xacro file ka path:** Upar diye gaye `humanoid_robot.xacro` file ko apne ROS 2 package ke `urdf` folder mein rakhen. `robot_state_publisher_node.py` file ko `your_package_name/your_package_name/` folder mein rakhen.

**Zaroori dependencies install karein:**
`pip install urdf_parser_py`

## 8. Humanoid Robot ko Visualize Karna (RViz integration)

RViz, ROS ka aik powerful 3D visualization tool hai, jo URDF files ka istemal karte hue robot models ko display karta hai. Jab `robot_state_publisher` node `/joint_states` topic par data publish karta hai, to RViz us data ko use karke robot ki joint positions ko update karta hai, jis se hum real-time mein robot ki harkat dekh sakte hain. RViz mein `RobotModel` display plugin add kar ke aur `Fixed Frame` ko `base_link` ya `odom` set kar ke aap apne robot ko visualize kar sakte hain.

## 9. Behtareen Tareeqe aur Advanced Topics

-   **Modular Design:** Hamesha Xacro macros ka istemal kar ke apne robot components ko modular rakhen.
-   **Parametrization:** Dimensions, masses, aur doosri properties ko Xacro properties ya arguments se define karein taake asani se changes ki ja saken.
-   **Collision aur Visual Geometry:** Hamesha collision aur visual geometry ko define karein. Collision geometry sim simple aur optimized honi chahiye simulation ke liye.
-   **Inertial Properties:** Sahi inertial properties (mass aur inertia tensor) provide karna physics-based simulations ke liye zaroori hai.
-   **Transmissions:** ROS control ke liye `transmission` tags ka istemal hota hai, jo joint aur motor ke darmiyan interface bayan karte hain.
-   **Gazebo Plugins:** Simulation environment (Gazebo) mein robot ke sensors aur actuators ko simulate karne ke liye Gazebo plugins URDF mein shamil kiye ja sakte hain.

## 10. Mutaddid Ikhtiyari Sawalaat (Multiple Choice Questions)

1.  URDF mein robot ke physical parts ko kis tag se bayan kiya jata hai?
    a)  `<joint>`
    b)  `<link>`
    c)  `<robot>`
    d)  `<visual>`

2.  Xacro ka bunyadi maqsad kya hai?
    a)  Robot ko control karna
    b)  URDF files ko modular aur reusable banana
    c)  Robot ke sensors se data parhna
    d)  Robot ko simulate karna

3.  Aik `fixed` joint ki type kya batati hai?
    a)  Joint ghoom sakta hai
    b)  Joint slide kar sakta hai
    c)  Do links hamesha ke liye jure hue hain, koi harkat nahi
    d)  Joint do axis par ghoom sakta hai

4.  `rclpy` mein `/joint_states` topic par data publish karne ka maqsad kya hai?
    a)  Robot ke motors ko control karna
    b)  RViz jaisi tools mein robot ko visualize karna
    c)  Robot ki battery level batana
    d)  Robot ko navigation ke liye istemal karna

5.  URDF mein `<inertial>` tag kis cheez ki tafseel deta hai?
    a)  Link ka rang
    b)  Link ka wazan aur mass distribution
    c)  Link ki shakal
    d)  Link ki collision properties
