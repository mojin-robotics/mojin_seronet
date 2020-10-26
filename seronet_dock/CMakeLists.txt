cmake_minimum_required(VERSION 2.8.3)
project(seronet_dock)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  scripts/smartsoft_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  set(robotlist
    cob4
  )
  find_package(roslaunch REQUIRED)
  foreach(robot ${robotlist})
    roslaunch_add_file_check(launch/seronet_dock.launch ROBOT=${robot})
  endforeach()
endif()
