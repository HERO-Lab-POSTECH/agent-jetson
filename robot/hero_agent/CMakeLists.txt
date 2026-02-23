cmake_minimum_required(VERSION 2.8.3)
project(hero_agent)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp sensor_msgs hero_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hero_agent
  CATKIN_DEPENDS std_msgs roscpp sensor_msgs hero_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(agent_main src/agent_main.cpp)
add_dependencies(agent_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agent_main ${catkin_LIBRARIES})

add_executable(agent_dvl src/agent_dvl.cpp)
add_dependencies(agent_dvl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agent_dvl ${catkin_LIBRARIES})

add_executable(agent_command
  src/agent_command.cpp
  src/darknet_control.cpp
  src/mosaic_survey.cpp
  src/teleop.cpp
)
add_dependencies(agent_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agent_command ${catkin_LIBRARIES})

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/key_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install targets
install(TARGETS
  agent_main
  agent_dvl
  agent_command
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
