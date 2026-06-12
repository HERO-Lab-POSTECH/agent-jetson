cmake_minimum_required(VERSION 2.8.3)
project(albc_rl)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  hero_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    hero_msgs
)

# Python-only package: install the node + helper scripts.
catkin_install_python(PROGRAMS
  scripts/rl_inference_node.py
  scripts/build_proprio.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
