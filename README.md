# WoLF: Whole-body Locomotion Framework for quadruped robots

```bash
ros2 launch wolf_controller wolf_controller_bringup.launch.xml robot_model:=spot robot_name:=ras_1
```

## Setup

see documentation [here](https://github.com/graiola/wolf-setup/blob/master/README.md)

## Installation

```bash
git clone https://github.com/BarbaraRatto/wolf.git

cd wold

git submodule update --init --recursive
```

## Compile

```bash
cd ~/ros2_ws #(to ros ws)
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release #(<-- will fail, need to install dependecies with command rosdep install)
source install/setup.bash
```
