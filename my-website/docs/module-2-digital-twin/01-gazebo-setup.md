# Module 2: Gazebo Setup

## Introduction to Gazebo

Gazebo is a powerful 3D robotics simulator widely used in the ROS ecosystem. It allows developers to accurately and efficiently test robot algorithms in a realistic virtual environment before deploying them to physical hardware. Gazebo provides a robust physics engine, high-quality graphics, and interfaces for sensors and actuators.

### Key Features:
-   **Physics Engine:** Simulates rigid body dynamics, gravity, friction, and collisions.
-   **Sensors:** Emulates various sensors like cameras, lidar, IMUs, and contact sensors.
-   **Actuators:** Provides interfaces for controlling robot joints and effectors.
-   **GUI:** An intuitive graphical user interface for visualizing the simulation, manipulating objects, and inspecting robot properties.
-   **Plugins:** Extensible architecture allowing users to create custom behaviors for robots and environments.

## Installation of Gazebo (ROS 2 Humble)

For ROS 2 Humble on Ubuntu, Gazebo Garden is typically used. Here's how to install it:

1.  **Set up your ROS 2 environment** (if you haven't already done so in Module 1).
    ```bash
sudo apt update
sudo apt install ros-humble-desktop
# Source your ROS 2 environment
source /opt/ros/humble/setup.bash
    ```

2.  **Install Gazebo Garden:**
    Gazebo is usually installed as part of the `ros-humble-desktop` meta-package. If you need to install it separately, or a specific version of Gazebo (e.g., Gazebo Classic or a newer version not yet default for Humble), you can use:
    ```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs # Installs Gazebo Garden along with ROS 2 integration packages
    ```
    To install Gazebo Classic (if needed for older projects):
    ```bash
sudo apt install ros-humble-gazebo-ros-pkgs # This will still give Gazebo Garden with Humble
# For Gazebo Classic (older, often used with ROS 1 or specific ROS 2 setups):
# Follow instructions at https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install
    ```

3.  **Verify Installation:**
    You can launch Gazebo to verify the installation:
    ```bash
gazebo
    ```
    This should open the Gazebo GUI. If it opens, the installation is successful.

## Basic Gazebo Workflow

1.  **Launch Gazebo:** Open the Gazebo GUI by typing `gazebo` in your terminal.
2.  **Load a World:** Gazebo comes with several pre-defined worlds (e.g., `empty.world`, `simple_room.world`). You can load them from the GUI or via the command line:
    ```bash
gazebo empty.world
    ```
3.  **Add Models:** Insert models (robots, objects) from the local library or online repositories directly into your world using the GUI.
4.  **Simulate:** Run the simulation, interact with robots, and observe their behavior.

## Integrating with ROS 2

ROS 2 nodes can interact with Gazebo through `gazebo_ros_pkgs`. These packages provide:
-   **ROS 2 interfaces:** For publishing sensor data (e.g., camera images, lidar scans) from Gazebo to ROS 2 topics.
-   **ROS 2 control:** For subscribing to ROS 2 topics to receive commands (e.g., joint velocities, robot poses) and apply them to simulated robots.
-   **URDF parsing:** Gazebo uses URDF (Unified Robot Description Format) and SDFormat (Simulation Description Format) to define robot models and worlds.

### Roman Urdu Explanation:

`Gazebo ek 3D robot simulator hai jahan aap apne robots ko computer mein chala kar test kar sakte hain. Is mein physics hoti hai, sensors kaam karte hain, aur aap robots ko control bhi kar sakte hain. Isko install karna seedha hai, aur phir aap apne virtual duniya bana kar robots ko chalate hain. ROS 2 ke saath mil kar yeh robots ko real-world ki tarah behave karata hai, jisse aap code ko physical robot par dalne se pehle sab theek kar sakte hain.`

### Multiple Choice Questions (MCQs):

1.  **What is the primary function of Gazebo in robotics development?**
    a) To write ROS 2 client libraries.
    b) To simulate robot behavior in a virtual 3D environment.
    c) To perform real-time robot control on physical hardware.
    d) To visualize ROS 2 communication graphs.
    *Correct Answer: b) To simulate robot behavior in a virtual 3D environment.*

2.  **Which ROS 2 distribution is typically associated with Gazebo Garden?**
    a) Foxy
    b) Galactic
    c) Humble
    d) Eloquent
    *Correct Answer: c) Humble*

3.  **Which command is used to launch the Gazebo GUI?**
    a) `ros2 run gazebo gazebo_gui`
    b) `gazebo`
    c) `gzclient`
    d) `ros2 launch gazebo_ros`
    *Correct Answer: b) `gazebo`*

4.  **What does `gazebo_ros_pkgs` primarily enable?**
    a) Developing Gazebo plugins in C++ only.
    b) Integrating ROS 2 nodes with Gazebo simulations.
    c) Designing 3D robot models without code.
    d) Controlling physical robots directly.
    *Correct Answer: b) Integrating ROS 2 nodes with Gazebo simulations.*

5.  **Which file format is commonly used to define robot models in Gazebo?**
    a) YAML
    b) JSON
    c) URDF (Unified Robot Description Format)
    d) XML
    *Correct Answer: c) URDF (Unified Robot Description Format)*

### Further Reading:
- [Gazebo Documentation](https://gazebosim.org/docs)
- [ROS 2 Gazebo Integration](https://docs.ros.org/en/humble/Tutorials/Simulators/Gazebo/Gazebo-ROS2-Integration.html)
