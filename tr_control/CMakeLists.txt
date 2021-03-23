cmake_minimum_required(VERSION 3.0.2)
project(tr_control)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS 
    nodes/tr_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
