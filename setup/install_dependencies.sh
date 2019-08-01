#!/bin/bash

ROS_DISTRO=kinetic

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update

cat ./ros_deps_list.txt | grep -v \# | xargs printf -- "ros-${ROS_DISTRO}-%s\n" | xargs sudo apt-get install -y

sudo rosdep init
rosdep update

sudo dpkg -i ./advr-superbuild*

# Setup Bashrc
if grep -Fwq "/opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc
then 
 	echo -e "${COLOR_INFO}Bashrc already updated, skipping this step...${COLOR_RESET}"
else
    	echo -e "${COLOR_INFO}Update the bashrc.${COLOR_RESET}"
	echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
fi

if grep -Fwq "/opt/ros/advr-superbuild/setup.bash" ~/.bashrc
then 
 	echo -e "${COLOR_INFO}Bashrc already updated, skipping this step...${COLOR_RESET}"
else
    	echo -e "${COLOR_INFO}Update the bashrc.${COLOR_RESET}"
	echo "/opt/ros/advr-superbuild/setup.bash" >> ~/.bashrc
fi

