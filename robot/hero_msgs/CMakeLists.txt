cmake_minimum_required(VERSION 2.8.3)
project(hero_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES
   hero_agent_state.msg
   hero_agent_sensor.msg
   hero_agent_dvl.msg
   hero_agent_thruster_cmd.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
