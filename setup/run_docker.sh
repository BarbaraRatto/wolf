#!/usr/bin/env bash

# Check args
if [ "$#" -ne 1 ]; then
  echo "usage: ./run.sh [IMAGE_NAME=wbc:latest]"
  IMAGE_NAME="wbc:latest"
else
  IMAGE_NAME=$1	
fi

# Get this script's path
pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd`
popd > /dev/null

set -e

# Hacky
xhost +local:docker

CONTAINER_NAME="wbc"

if [ `sudo systemctl is-active docker` = "inactive" ]; then
  echo "Docker inactive.  Starting docker..."
  sudo systemctl start docker
fi

if [ "$(docker container inspect $CONTAINER_NAME > /dev/null 2>&1)" ]; then
	docker rm $CONTAINER_NAME
fi


# Run the container with shared X11
#--entrypoint "eval $(/usr/bin/ssh-agent -s) /usr/bin/ssh-add /home/`whoami`/.ssh/id_rsa"
docker run --user `id -u`:sudo --hostname $HOSTNAME --device=/dev/dri:/dev/dri --privileged -e "QT_X11_NO_MITSHM=1" -e SHELL -e DISPLAY -e DOCKER=1 --name $CONTAINER_NAME \
--gpus all \
--device=/dev/ttyUSB0 \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--workdir="/home/$USER" \
--volume="/etc/group:/etc/group:ro" \
--volume="/etc/passwd:/etc/passwd:ro" \
--volume="/etc/shadow:/etc/shadow:ro" \
--volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
--volume="$HOME/.ros:$HOME/.ros" \
--volume="$HOME/.gazebo:$HOME/.gazebo" \
--volume="$HOME/.ignition:$HOME/.ignition" \
-it $IMAGE_NAME $SHELL -c "eval export HOME=$HOME; cd $HOME; source /opt/ros/melodic/setup.bash; source /opt/ros/advr-superbuild/setup.bash; roslaunch wb_controller wb_controller_bringup.launch"
