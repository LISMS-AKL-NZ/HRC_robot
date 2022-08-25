# ros control

## dependencies
```bash

pip install -r requirements # for whole robot

```
follow instructions in https://github.com/cambel/ur_ikfast # for ur_kinematics

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
roslaunch azure_kinect_ros_driver driver.launch

roslaunch yolo_run yolov5.launch

roslaunch ur_robot_driver ur5e_bringup.launch

roslaunch robotiq_85_bringup robotiq_85.launch

roslaunch robotiq_ft_sensor ft_sensor.launch

roslaunch vention_conveyor_bringup vention_conveyor.launch 

roslaunch pycontrol ur5e_workbench.launch

rosrun pycontrol ros_demo.py
```
