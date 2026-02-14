cmake_minimum_required(VERSION 2.8.3)
project(hero_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
)

add_service_files(
  DIRECTORY srv
  FILES
  hero_command.srv
  )
## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES
   hero_agent_state.msg
   hero_agent_sensor.msg
   hero_usbl_cont.msg
   hero_xy_cont.msg
   hero_agent_vision.msg
   hero_agent_dvl.msg
   hero_agent_cont_xy.msg
   hero_agent_position_result.msg
   hero_agent_cont_para.msg
   hero_agent_dvl_velocity.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
