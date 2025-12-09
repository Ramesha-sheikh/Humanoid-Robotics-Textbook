---
sidebar_position: 1
title: NVIDIA Isaac Sim Installation
---

# NVIDIA Isaac Sim Installation

NVIDIA Isaac Sim, built on the NVIDIA Omniverse platform, is a powerful and scalable robotics simulation application that serves as an essential tool for synthetic data generation. It offers a robust environment for developing, testing, and training AI-powered robots. This chapter provides a comprehensive guide to installing Isaac Sim, detailing prerequisites, system requirements, and step-by-step setup instructions.

## Prerequisites

Before you begin the installation, ensure your system meets the following prerequisites:

*   **NVIDIA GPU:** An NVIDIA RTX series GPU is highly recommended for optimal performance. While some other NVIDIA GPUs may function, an RTX card ensures the best experience for complex simulations and real-time rendering.
*   **Operating System:**
    *   **Ubuntu Linux:** Ubuntu 20.04 LTS or 22.04 LTS is officially supported and highly recommended.
    *   **Windows:** Windows users can run Isaac Sim via [Windows Subsystem for Linux (WSL2)](https://docs.microsoft.com/en-us/windows/wsl/install). Ensure WSL2 is properly set up with an Ubuntu distribution, has GPU passthrough enabled, and a graphical environment (like a VcXsrv or Wayland compositor) is configured for UI usage.
*   **NVIDIA Driver:** Ensure you have the latest stable NVIDIA proprietary drivers installed. For Ubuntu, this typically means a driver version of 515 or newer (e.g., 535, 545, etc.).
*   **Docker and NVIDIA Container Toolkit:** Isaac Sim leverages Docker containers for deployment. You will need to install Docker and the NVIDIA Container Toolkit.

## System Requirements

The following are the minimum and recommended system specifications for running NVIDIA Isaac Sim:

| Component       | Minimum Requirement                                   | Recommended Specification                                  |
| :-------------- | :---------------------------------------------------- | :--------------------------------------------------------- |
| **CPU**         | Intel Core i7-8700 or AMD Ryzen 7 3700X               | Intel Core i9-10900K or AMD Ryzen 9 5900X                  |
| **RAM**         | 32 GB                                                 | 64 GB or more                                              |
| **GPU**         | NVIDIA GeForce RTX 2060 or Quadro RTX 4000 (8 GB VRAM) | NVIDIA GeForce RTX 3080 or RTX 4080, or Quadro RTX 6000 (10 GB+ VRAM) |
| **Storage**     | 500 GB SSD                                            | 1 TB NVMe SSD                                              |
| **Operating System** | Ubuntu 20.04/22.04 LTS or Windows 10/11 (with WSL2) | Ubuntu 20.04/22.04 LTS                                     |

## Step-by-Step Installation

Follow these steps carefully to install NVIDIA Isaac Sim on your system. These instructions are primarily tailored for an Ubuntu Linux environment; Windows users utilizing WSL2 should adapt the commands as necessary.

### Step 1: Install NVIDIA Drivers

Ensure your NVIDIA drivers are up to date.

```bash
sudo apt update
sudo apt install -y ubuntu-drivers-common
sudo ubuntu-drivers autoinstall
# Reboot if prompted
sudo reboot
```

Verify your driver installation:

```bash
nvidia-smi
```

You should see output detailing your NVIDIA GPU and driver version.

### Step 2: Install Docker and NVIDIA Container Toolkit

Isaac Sim runs inside a Docker container, requiring both Docker and the NVIDIA Container Toolkit.

#### Install Docker Engine

```bash
# Add Docker's official GPG key:
sudo apt update
sudo apt install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update

# Install Docker Engine, containerd, and Docker Compose
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Verify Docker installation:

```bash
sudo docker run hello-world
```

Add your user to the `docker` group to run Docker commands without `sudo`:

```bash
sudo usermod -aG docker $USER
newgrp docker # Apply group changes immediately (session-specific; a full re-login is required for permanent effect)
```

#### Install NVIDIA Container Toolkit

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/ubuntu20.04/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker
```

Verify NVIDIA Container Toolkit installation:

```bash
sudo docker run --rm --gpus all nvidia/cuda:11.7.1-base-ubuntu20.04 nvidia-smi
```

You should see `nvidia-smi` output from within the Docker container.

### Step 3: Download and Extract Isaac Sim

NVIDIA Isaac Sim is typically downloaded via the NVIDIA Omniverse Launcher or directly as a tarball for HPC/headless deployments. For this guide, we will use the tarball method.

1.  **Download Isaac Sim:** Navigate to the [NVIDIA Omniverse website](https://developer.nvidia.com/omniverse/downloads). From there, look for the 'Isaac Sim' product page or the 'HPC/Headless' download section to obtain the Linux tarball. Ensure you download the version compatible with your system and requirements.
    *   *Note: An NVIDIA Developer account is usually required to access these downloads.*
2.  **Extract the Tarball:** Once downloaded (e.g., `Isaac_Sim_2023.1.1.tar.gz`), move it to your desired installation directory (e.g., `/opt/nvidia`).

    ```bash
    sudo mkdir -p /opt/nvidia
    sudo mv ~/Downloads/Isaac_Sim_2023.1.1.tar.gz /opt/nvidia/
    cd /opt/nvidia
    sudo tar -xvf Isaac_Sim_2023.1.1.tar.gz
    # This will extract Isaac Sim into a directory like `isaac_sim-2023.1.1` (the exact name may vary by version).

### Step 4: Run Isaac Sim

Now you can launch Isaac Sim.

```bash
cd /opt/nvidia/isaac_sim-<VERSION> # Navigate to your extracted Isaac Sim directory, replacing <VERSION> with your installed version (e.g., isaac_sim-2023.1.1)
./run_headless.sh # Or ./isaac_sim.sh for UI version, if available
```

The first run might take some time as it downloads necessary Docker images and sets up the environment.

### Step 5: Verification

Once Isaac Sim is running, you can verify the installation by:

*   **Headless:** Checking the log output for any errors and confirmation messages.
*   **UI Version:** If you launched with UI, opening a sample scene (e.g., from the `isaac_sim/_build/linux-x86_64/release/data/usd_samples` directory or through the UI examples).

## Conclusion

You have successfully installed NVIDIA Isaac Sim and are now prepared to leverage its capabilities for advanced robotics simulation, synthetic data generation, and AI model training. For further details on advanced usage, development, and specific tutorials, consult the official NVIDIA Isaac Sim documentation.
