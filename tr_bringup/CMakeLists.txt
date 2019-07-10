cmake_minimum_required(VERSION 2.8.3)
project(tr_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS 
    scripts/tr_system
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})