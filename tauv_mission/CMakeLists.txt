cmake_minimum_required(VERSION 2.8.3)
project(tauv_mission)

find_package(catkin REQUIRED COMPONENTS
  std_msgs 
  message_generation)

catkin_python_setup()

## Generate services in the 'srv' folder


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

catkin_install_python(PROGRAMS scripts/run_qualification.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
