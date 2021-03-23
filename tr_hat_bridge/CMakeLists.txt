cmake_minimum_required(VERSION 3.0.2)
project(tr_hat_bridge)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS 
    nodes/tr_hat_bridge
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
