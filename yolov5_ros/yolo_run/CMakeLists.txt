cmake_minimum_required(VERSION 3.0.2)
project(yolo_run)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roslaunch
)

# catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS rospy std_msgs
)

roslaunch_add_file_check(launch)

catkin_install_python(PROGRAMS src/detect.py
  DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION)
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Install ## 
install(DIRECTORY launch src
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
