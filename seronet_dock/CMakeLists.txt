cmake_minimum_required(VERSION 2.8.3)
project(seronet_dock)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package()

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  scripts/smartsoft_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
