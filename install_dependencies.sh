ROS_DISTRO=kinetic

echo -e "${COLOR_INFO}Installing ROS-Control...${COLOR_RESET}"
sudo apt-get -y install ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers ros-${ROS_DISTRO}-realtime-tools ros-${ROS_DISTRO}-gazebo-ros-control
