# ros control


## run following commands in sequence
catkin_make

roslaunch ur_robot_driver ur5e_bringup.launch

rosrun ur_robot_driver tool_communication

roslaunch robotiq_85_bringup robotiq_85.launch

rosrun robotiq_ft_sensor rq_sensor

rosrun pycontrol ros_cart


## building
```bash

# create a catkin workspace
$ mkdir -p robot_ws/src && cd robot_ws

# clone the project
$ git clone https://github.com/WanqingXia/HRC_robot.git

# install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# build the workspace
$ catkin_make

# activate the workspace (ie: source it)
$ source devel/setup.bash
```
