# ros control
Follow the instruction step by step to build the project, system requirement: Ubuntu 20.04

## Clone the project
```bash
# create a catkin workspace
mkdir -p robot_ws/src && cd robot_ws/src

# clone the project noetic branch
git clone -b ros-noetic https://github.com/WanqingXia/HRC_robot.git .
```

## Dependencies
### For ur_ikfast package
Original from https://github.com/cambel/ur_ikfast
```
sudo apt-get install libblas-dev liblapack-dev

cd ur_ikfast # install ur_ikfast as python package
pip install -e .
```

### For using Azure Kinect Sensor SDK
Original from https://gist.github.com/madelinegannon/c212dbf24fc42c1f36776342754d81bc
1. Add source using curl
```bash
sudo apt install curl
curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
```
2. Modify `/etc/apt/sources.list`. At the bottom of the file, change from:
```
deb https://packages.microsoft.com/ubuntu/18.04/prod bionic main
# deb-src https://packages.microsoft.com/ubuntu/18.04/prod bionic main
```
 to:
```
deb [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main
# deb-src [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main
```

3. Rerun `sudo apt-get update`

4. Install Kinect Packages

```
sudo apt install k4a-tools
sudo apt install libk4a1.4-dev
```

5. Finish Device Setup

[Finish device setup](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#linux-device-setup) by setting up udev rules:

- Copy '[scripts/99-k4a.rules](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules)' into '/etc/udev/rules.d/'.
- Detach and reattach Azure Kinect devices if attached during this process.

6. Verify installation by call it directly in the terminal:
```
k4aviewer
```

### For Python packages
```bash
pip install -r requirements.txt # install all python packages
```

### For ROS packages

```bash
# install dependencies
sudo apt update -qq
rosdep update
cd /robot_ws
rosdep install --from-paths src/Universal_Robots_ROS_Driver/ --ignore-src -y
rosdep install --from-paths src/fmauch_universal_robot/ --ignore-src -y
rosdep install --from-paths src/robotiq_85_gripper/ --ignore-src -y
```

## Building the project
```
# default building type is RelWithDebInfo, set in the toplevel CMakeList.txt
catkin build 

# activate the workspace (ie: source it)
echo 'source $HOME/$USER/robot_ws/devel/setup.bash' >> ~/.bashrc 
source ~/.bashrc
```

## Run following commands in sequence
```bash
roslaunch azure_kinect_ros_driver driver.launch

roslaunch yolo_run yolov5.launch

roslaunch ur_robot_driver ur5e_bringup.launch

roslaunch robotiq_85_bringup robotiq_85.launch

roslaunch robotiq_ft_sensor ft_sensor.launch

roslaunch vention_conveyor_bringup vention_conveyor.launch 

rviz -d src/workbench/workbench_bringup/launch/combine.rviz

rosrun pycontrol ros_demo.py
```

## Trouble shooting
1. Having trouble with communicate with robot
```
# disable firewall to communicate with robot
sudo ufw disable
```

2. Having trouble with controlling the gripper
```
# change permission for /tmp/ttyUR
sudo chmod -t /tmp
sudo chmod 777 /tmp/ttyUR
sudo chmod +t /tmp
```

3. Having trouble with controlling the Force/Torque sensor
```
# change permission for /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB0
```