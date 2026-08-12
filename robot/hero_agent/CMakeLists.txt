cmake_minimum_required(VERSION 2.8.3)
project(hero_agent)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp hero_msgs)

# No LIBRARIES here: this package builds only the `agent` executable below, so
# exporting a `hero_agent` library promises a target that is never created. A
# warm devel space hid that -- on a clean build the promise surfaces as
# "tried to find library 'hero_agent' ... neither a target nor built/installed"
# in every downstream find_package (characterization_tests). albc_control is the
# correct model next door: INCLUDE_DIRS only, headers arrive via
# catkin_INCLUDE_DIRS.
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs roscpp hero_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(agent src/agent.cpp)
add_dependencies(agent ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agent ${catkin_LIBRARIES})

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/key_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install targets
install(TARGETS
  agent
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
