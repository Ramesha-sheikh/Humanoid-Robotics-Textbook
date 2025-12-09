---
sidebar_position: 1
sidebar_label: Installation Guide
---

# Installation Guide

This guide provides comprehensive instructions for setting up your development environment for Physical AI and Humanoid Robotics. It covers essential software such as ROS 2, NVIDIA Isaac Sim, and other crucial libraries and tools. This guide assumes you have a basic understanding of operating systems and command-line interfaces but are new to this specific robotics and simulation stack.

## 1. System Requirements

Before you begin, ensure your system meets the following minimum requirements:

*   **Operating System**: Ubuntu 20.04 (Focal Fossa) or 22.04 (Jammy Jellyfish) LTS is highly recommended for ROS 2 and Isaac Sim compatibility. Windows 10/11 with WSL2 (Windows Subsystem for Linux) can also be used for ROS 2, but native Linux is preferred for Isaac Sim.
*   **CPU**: Intel i7/Xeon or equivalent AMD processor (8 cores or more recommended).
*   **RAM**: 32 GB or more.
*   **GPU**: NVIDIA RTX series GPU (RTX 3060 or higher recommended) with the latest proprietary NVIDIA drivers installed. Isaac Sim heavily relies on NVIDIA GPUs.
*   **Storage**: 500 GB SSD or more, with at least 100 GB free for installations.
*   **Internet Connection**: Stable and fast internet connection for downloading large packages.

## 2. ROS 2 Installation (Humble Hawksbill / Iron Irwini)

We recommend installing either ROS 2 Humble Hawksbill (for Ubuntu 20.04) or Iron Irwini (for Ubuntu 22.04). Choose the distribution that matches your Ubuntu version.

### 2.1. Set up Locale

Ensure you have a UTF-8 locale.

```bash
locale # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale # verify again
```

### 2.2. Add ROS 2 apt Repository

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

For Ubuntu 20.04 (Humble Hawksbill):

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

For Ubuntu 22.04 (Iron Irwini):

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2.3. Install ROS 2 Packages

Update your apt repository cache and install the ROS 2 Desktop Full.

```bash
sudo apt update
sudo apt upgrade -y

# For Humble Hawksbill (Ubuntu 20.04)
sudo apt install ros-humble-desktop-full -y

# For Iron Irwini (Ubuntu 22.04)
sudo apt install ros-iron-desktop-full -y
```

### 2.4. Environment Setup

Source the ROS 2 setup script in your shell. For convenience, add it to your `~/.bashrc` or `~/.zshrc`.

```bash
# Replace '<YOUR_ROS2_DISTRO>' with 'humble' or 'iron' depending on your installation
echo "source /opt/ros/<YOUR_ROS2_DISTRO>/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/<YOUR_ROS2_DISTRO>/" >> ~/.bashrc
echo "source /usr/share/ament_auto_prefix_bash/ament_auto_prefix_bash.sh" >> ~/.bashrc
source ~/.bashrc
```

### 2.5. Install Dependencies for `rosdep`

```bash
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
```

### 2.6. Test Your ROS 2 Installation

Open two separate terminal windows.

**Terminal 1 (Talker):**

```bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2 (Listener):**

```bash
ros2 run demo_nodes_py listener
```

You should see messages being exchanged between the talker and listener nodes.

## 3. NVIDIA Isaac Sim Installation

NVIDIA Isaac Sim is a powerful robotics simulation platform built on NVIDIA Omniverse.

### 3.1. NVIDIA Driver Installation

Ensure you have the latest NVIDIA drivers installed. For Ubuntu, you can typically install them via:

```bash
sudo ubuntu-drivers autoinstall
sudo reboot # Reboot after driver installation
```

Verify installation:

```bash
nvidia-smi
```

### 3.2. Install NVIDIA Omniverse Launcher

Isaac Sim is installed and managed via the NVIDIA Omniverse Launcher.

1.  Download the Omniverse Launcher from the [NVIDIA Omniverse website](https://www.nvidia.com/omniverse/download/launcher/).
2.  Follow the installation instructions provided on the website for your specific OS.
3.  Once installed, open the Omniverse Launcher and log in with your NVIDIA account.

### 3.3. Install Isaac Sim

1.  In the Omniverse Launcher, navigate to the "Exchange" tab.
2.  Search for "Isaac Sim" and click "Install".
3.  Choose an installation path. It's recommended to install it on an SSD.
4.  After installation, click "Launch" to start Isaac Sim. The first launch might take some time as it downloads additional assets.

### 3.4. Isaac Sim ROS 2 Bridge Setup

Isaac Sim includes a ROS 2 bridge for seamless integration with your ROS 2 environment.

1.  Open Isaac Sim.
2.  Go to "Window" -> "Extensions" and enable the "ROS 2 Bridge" extension.
3.  Refer to the official Isaac Sim documentation for detailed instructions on setting up and using the ROS 2 bridge, including sourcing the `setup_ros.bash` script from your Isaac Sim installation directory.

    ```bash
    # Example path, adjust according to your Isaac Sim installation directory. Common path is ~/.local/share/ov/pkg/isaac_sim-202X.X.X/ros2_ws/install/setup.bash
    source ~/.bashrc
    source /path/to/your/isaac-sim-installation/ros2_ws/install/setup.bash
    ```

## 4. Essential Libraries and Tools

### 4.1. Build Tools

```bash
sudo apt install build-essential cmake git python3-pip -y
```

### 4.2. Python Development Tools

```bash
sudo apt install python3-dev python3-venv -y
pip install --upgrade pip
```

### 4.3. VS Code (Recommended IDE)

Visual Studio Code is a popular IDE with excellent support for C++, Python, and ROS 2 development.

1.  Download the `.deb` package from the [VS Code website](https://code.visualstudio.com/download).
2.  Install using dpkg:

    ```bash
    sudo dpkg -i code_*.deb
    sudo apt install --fix-broken -y # To fix any dependency issues
    ```
3.  Install useful extensions:
    *   ROS
    *   C/C++ Extension Pack
    *   Python
    *   Pylance
    *   GitLens

### 4.4. Other Useful Tools

```bash
sudo apt install htop ncdu tree net-tools -y
```

## 5. Verification

After completing all installations, perform the following checks:

*   **ROS 2**: Run `ros2 topic list` and `ros2 node list` to ensure ROS 2 is functional.
*   **Isaac Sim**: Launch Isaac Sim and try running one of the built-in robotics examples.
*   **Python**: Verify Python and pip are working: `python3 --version` and `pip --version`.
*   **VS Code**: Open VS Code and ensure you can open and edit `.cpp` and `.py` files with syntax highlighting.

This concludes the general installation guide. You are now ready to begin your journey into Physical AI and Humanoid Robotics!
