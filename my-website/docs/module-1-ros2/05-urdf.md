# ROS 2: URDF (Unified Robot Description Format)

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML format used in ROS to describe all aspects of a robot. It allows you to define the robot's kinematic and dynamic properties, visual appearance, and collision properties. URDF files are essential for simulating robots in tools like Gazebo, visualizing them in RViz, and performing motion planning.

### Key Concepts:
- **Link:** Represents a rigid body segment of the robot (e.g., a base, a limb, a wheel).
- **Joint:** Connects two links and defines their relative motion (e.g., revolute, prismatic, fixed).
- **Kinematics:** Describes the motion of the robot without considering the forces that cause the motion (position, velocity, acceleration).
- **Dynamics:** Describes the motion of the robot considering forces and torques (mass, inertia).
- **Visual:** Defines the graphical mesh or shape of a link for visualization.
- **Collision:** Defines the collision mesh or shape of a link for physics simulation.

### Structure of a URDF File:

A URDF file starts with a `<robot>` tag and contains multiple `<link>` and `<joint>` tags.

```xml
<?xml version="1.0"?>
<robot name="my_simple_robot">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="base_to_wheel_joint" type="revolute">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.2 0 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2" upper="2" effort="100" velocity="100"/>
  </joint>

</robot>
```

### Explanation of Tags:
- `<robot name="...">`: The root element of the URDF file.
- `<link name="...">`: Defines a rigid body with its `visual`, `collision`, and `inertial` properties.
  - `<visual>`: Specifies the visual representation of the link.
  - `<collision>`: Specifies the collision model of the link, used for physics interactions.
  - `<inertial>`: Defines the mass and inertia properties of the link.
- `<joint name="..." type="...">`: Connects two links and defines their relative motion.
  - `<parent link="..."/>`: Specifies the parent link.
  - `<child link="..."/>`: Specifies the child link.
  - `<origin xyz="..." rpy="..."/>`: Defines the pose (position and orientation) of the child link relative to the parent link.
  - `<axis xyz="..."/>`: Specifies the axis of rotation for revolute joints or translation for prismatic joints.
  - `<limit lower="..." upper="..." effort="..." velocity="..."/>`: Defines the joint limits.

### Xacro (XML Macros for ROS):

For more complex robots, URDF files can become very long and repetitive. Xacro (XML Macros for ROS) is a macro language that allows you to use variables, mathematical expressions, and conditional statements to create more concise and readable robot descriptions. Xacro files are typically processed into URDF files before being used by ROS tools.

### Further Reading:
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [Xacro Overview](http://wiki.ros.org/xacro)
