cmake_minimum_required(VERSION 2.8.3)
project(albc_rl)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  hero_msgs
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/GyroOffset.cfg
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    hero_msgs
    dynamic_reconfigure
)

# Python-only package: install the node + helper scripts.
catkin_install_python(PROGRAMS
  scripts/rl_inference_node.py
  scripts/thruster_mixer.py
  scripts/b1_channel_probe.py
  scripts/hold_joint.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY numpy_port/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/numpy_port)
install(FILES scripts/deployed_tam.json DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
