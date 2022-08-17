# ros control

## dependencies
```bash
pip install scipy

pip install socketIO-client

pip install pathlib

pip install paho-mqtt
```

## building
```bash

# create a catkin workspace
mkdir -p robot_ws/src && cd robot_ws

# clone the project
git clone https://github.com/WanqingXia/HRC_robot.git

# install dependencies
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# activate the workspace (ie: source it)
source devel/setup.bash
```

## run following commands in sequence
```bash
roslaunch yolo_run yolov5.launch

roslaunch ur_robot_driver ur5e_bringup.launch

roslaunch robotiq_85_bringup robotiq_85.launch

rosrun robotiq_ft_sensor rq_sensor

rosrun vention_conveyor_driver conveyor_launch.py

rosrun pycontrol ros_cart.py
```
