cmake_minimum_required(VERSION 2.8.3)
project(tele_dir)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(
   PROGRAMS src/TeleDir.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
