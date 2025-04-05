before starting rename the cloned package to "marketcart" to make the commands below work

#simulation package launch 

ros2 launch cart_navigation navigation.launch.py #navigation with robot spawned
ros2 launch cart_navigation mapping.launch.py #does the slam
ros2 launch cart_navigation cart.launch.py #spawn the robot

#hardware package launch 

ros2 launch hardware_navigation navigation.launch.py #navigation with robot spawned
ros2 launch hardware_navigation mapping.launch.py #does the slam
ros2 launch hardware_navigation cart.launch.py #spawn the robot

#launching follow me node
cd ~/ros2_ws/src/marketmart/cart_navigation/detection
detect.py

#launching follow me node
cd ~/ros2_ws/src/marketmart/cart_navigation/src/cart_navigation
python3 follow_me_node.py

#launching the gui
cd ~/ros2_ws/src/marketmart/cart_navigation/src/cart_navigation
python3 gui.py


#all required dependancies for running the packages
sudo rosdep init
rosdep update

sudo apt update
sudo apt install '~nros-<distro>-rqt*'
rqt

#gazebo install
sudo apt-get install ros-${ROS_DISTRO}-ros-gz -y  

#Install useful tools
sudo apt-get install python3-vcstool
sudo apt-get install python3 python3-pip -y
sudo apt-get install python3-numpy

sudo apt-get install ros-${ROS_DISTRO}-urdf-tutorial

#IMU
sudo apt install ros-<distro>-rviz-imu-plugin

#Localization
sudo apt install ros-<distro>-robot-localization

#Slam & Navigation
sudo apt install ros-jazzy-slam-toolbox
# following are optional
sudo apt install ros-<ros2-distro>-navigation2
sudo apt install ros-<ros2-distro>-nav2-bringup

#server connection
sudo apt install ros-jazzy-nav2-map-server