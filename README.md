# 🐢 ROS Humble BlueRov2 Driver 🌊

<p align="center">
  <img src="img/banner.jpg" />
</p>

## What is possible ?
- 📷 Video streaming using OpenCV 
- 📜 Reading and writing data via the MAVLink protocol with PYMAVLINK
- 🎮 Joystick interaction 
- 🤖 PID controller for automatic depth control and yaw, pitch, roll orientation 
- 🌊 Controlling the [Tritech Micron](https://www.tritech.co.uk/products/micron-sonar) sonar and outputting data as a point cloud 

## Getting Started
In this guide, we'll walk you through the process of setting up your development environment to work with ROS 2 Humble and a specific project repository on Ubuntu Jammy Jellyfish (22.04). Follow these steps to get started:

### 1. Install Ubuntu Jammy Jellyfish (22.04)
To begin, you'll need to install Ubuntu Jammy Jellyfish (22.04) on your computer. You can find installation instructions on the [Ubuntu website](https://ubuntu.com/download/desktop) or choose a suitable installation method for your system.

### 2. Install ROS 2 Humble
Next, you'll need to install ROS 2 Humble. Follow the official ROS 2 installation guide for Ubuntu development setup available at [ROS Humble](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) to install ROS 2 on your system.

### 3. Clone the Project Repository
Now that you have Ubuntu Jammy Jellyfish and ROS 2 Humble installed, you can proceed to clone the project repository to your computer. Open a terminal and run the following command:

```bash
git clone https://github.com/nebverse/ROS-Humble-BlueRov2-Driver.git -b nebverse
```

This will download the project's source code to your local machine.

### 4. Install ROS Dependencies
Navigate to your ROS workspace's root directory and install the ROS dependencies using ROS 2's package manager, ***'rosdep'*** :

```bash
cd ~/ROS-Humble-BlueRov2-Driver
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

### 5. Check Your BlueRov Configuration
Before you proceed further with the ROS 2 project, it's important to ensure that your BlueRov is configured correctly. Follow these steps to verify your BlueRov's configuration:

#### 5.1 BlueOS Installation

Make sure that your BlueRov is running "BlueOS" or an equivalent operating system that supports ROS 2 integration. BlueOS is a popular choice for ROVs and provides the necessary infrastructure to work with ROS 2.

#### 5.2 Mavlink Communication

ROS 2 relies on Mavlink communication to interact with your BlueRov. Ensure that your BlueRov is configured to establish Mavlink communication with your development environment. This typically involves configuring the communication settings on your ROV's companion computer or onboard computer.

## Control Layout
| **Control**              | **Gamepad Input**        | **Description**                                                | **Published Topic**       |
|---------------------------|--------------------------|----------------------------------------------------------------|---------------------------|
| **Forward/Backward**      | Left Joystick (Up/Down)  | Move the ROV forward or backward.                             | `/bluerov2/rc/forward`    |
| **Lateral Movement**      | Left Joystick (Left/Right)| Strafe the ROV left or right.                                  | `/bluerov2/rc/lateral`    |
| **Yaw (Rotate)**          | Right Joystick (Left/Right)| Rotate the ROV left or right.                                 | `/bluerov2/rc/yaw`        |
| **Throttle (Depth)**      | Right Joystick (Up/Down) | Adjust the ROV's depth (dive/surface).                        | `/bluerov2/rc/throttle`   |
| **Camera Pan**            | D-Pad (Left/Right)      | Pan the camera horizontally (left/right).                     | `/bluerov2/rc/camera_pan` |
| **Camera Tilt**           | D-Pad (Up/Down)         | Tilt the camera vertically (up/down).                         | `/bluerov2/rc/camera_tilt`|
| **Lights Intensity Up**   | Right Bumper (RB)        | Increase the intensity of the lights.                         | `/bluerov2/rc/lights`     |
| **Lights Intensity Down** | Left Bumper (LB)         | Decrease the intensity of the lights.                         | `/bluerov2/rc/lights`     |
| **Gripper Open/Close**    | A Button                | Toggle the gripper between open and close states.             | `/bluerov2/rc/gripper`    |
| **Arm/Disarm Thrusters**  | Start Button             | Toggle between arming and disarming the ROV thrusters.         | `/bluerov2/arm`           |

