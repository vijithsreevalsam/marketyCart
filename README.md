# Smart Supermarket Cart – Simulation & Hardware Integration

This repository contains the complete ROS 2 implementation of the Smart Cart project, featuring autonomous navigation, Follow Me mode, object detection via YOLO, and a custom PyQt5 GUI for user control.

> **Note:** After cloning the repository, please rename the folder to `marketcart` to ensure the launch commands below work as expected.

---

## 🧪 Simulation Launch Commands

These commands are for testing the robot in Gazebo with the full navigation stack.

```bash
# Launch full navigation stack with robot spawned
ros2 launch cart_navigation navigation.launch.py

# Launch SLAM (for map creation)
ros2 launch cart_navigation mapping.launch.py

# Only spawn the robot in Gazebo
ros2 launch cart_navigation cart.launch.py

🤖 Hardware Launch Commands

These are to be used when testing on the real robot with hardware integration.

# Launch navigation with robot and hardware nodes
ros2 launch hardware_navigation navigation.launch.py

# Launch SLAM using real sensors
ros2 launch hardware_navigation mapping.launch.py

# Spawn robot model without SLAM
ros2 launch hardware_navigation cart.launch.py

⚙️ Running Additional Features
Follow Me Node

cd src/marketcart/cart_navigation/src/cart_navigation
python3 follow_me_node.py

Object Detection (YOLOv8)

cd src/marketcart/cart_navigation/detection
python3 detect.py

Launch the GUI

cd src/marketcart/cart_navigation/src/cart_navigation
python3 gui.py

🧩 Required Dependencies

Before running the project, install the following dependencies:
Initialize rosdep

sudo rosdep init
rosdep update

General Dependencies

sudo apt update
sudo apt install '~nros--rqt*' rqt
sudo apt install python3 python3-pip python3-numpy
sudo apt install python3-vcstool
sudo apt install ros-${ROS_DISTRO}-urdf-tutorial

Gazebo Integration

sudo apt install ros-${ROS_DISTRO}-ros-gz -y

IMU Visualization

sudo apt install ros-${ROS_DISTRO}-rviz-imu-plugin

Localization

sudo apt install ros-${ROS_DISTRO}-robot-localization

SLAM & Navigation

sudo apt install ros-${ROS_DISTRO}-slam-toolbox
sudo apt install ros-${ROS_DISTRO}-navigation2
sudo apt install ros-${ROS_DISTRO}-nav2-bringup
sudo apt install ros-${ROS_DISTRO}-nav2-map-server

    ⚠️ Replace ${ROS_DISTRO} with your ROS 2 version (e.g., jazzy)

📝 Notes

    All simulation work (SLAM, AMCL, Follow Me, and GUI) has been tested with ROS 2 Jazzy and Gazebo.

    This project includes both simulation and hardware support, but simulation features are fully functional for demonstration purposes.

    Contribution credits and presentation responsibilities are handled separately as part of the final submission.

👤 Author

Name: Amen Ahmed
Email: AA5508@lice.mdx.ac.uk
Submission: Final Submission 2 – Simulation & GUI Lead