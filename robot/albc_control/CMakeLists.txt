cmake_minimum_required(VERSION 2.8.3)
project(albc_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_sdk
  hero_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    dynamixel_sdk
    hero_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Dynamixel driver
add_executable(joint_angle_command src/joint_angle_command.cpp)
add_dependencies(joint_angle_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_angle_command ${catkin_LIBRARIES})

# ALBC controller (damped least squares)
add_executable(albc_controller src/albc_controller.cpp)
add_dependencies(albc_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(albc_controller ${catkin_LIBRARIES})

install(TARGETS
  joint_angle_command
  albc_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
