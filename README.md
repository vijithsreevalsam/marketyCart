#all required dependancies for running the packages


sudo rosdep init
rosdep update

sudo apt update
sudo apt install '~nros-jazzy-rqt*'
rqt

#gazebo install
sudo apt-get install ros-${ROS_DISTRO}-ros-gz -y  

#Install useful tools
sudo apt-get install python3-vcstool
sudo apt-get install python3 python3-pip -y
sudo apt-get install python3-numpy

sudo apt-get install ros-${ROS_DISTRO}-urdf-tutorial

#IMU
sudo apt install ros-jazzy-rviz-imu-plugin

#Localization
sudo apt install ros-jazzy-robot-localization

#Slam & Navigation
sudo apt install ros-jazzy-slam-toolbox
# following are optional
sudo apt install ros-<ros2-distro>-navigation2
sudo apt install ros-<ros2-distro>-nav2-bringup

#server connection
sudo apt install ros-jazzy-nav2-map-server