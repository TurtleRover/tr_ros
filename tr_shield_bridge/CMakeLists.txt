cmake_minimum_required(VERSION 2.8.3)
project(tr_shield_bridge)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/tr_shield_bridge
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
