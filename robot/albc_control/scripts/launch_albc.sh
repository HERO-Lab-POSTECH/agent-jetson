#!/bin/bash
# launch_albc.sh — Start ALBC controller with interactive mode selection
#
# Usage: launch-albc  (alias this script)
#
# Runs joint_angle_command in background, albc_controller in foreground
# so that albc_controller gets stdin for key input (mode selection + runtime).

set -e

# ROS environment
source /opt/ros/lunar/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash

# Load parameters to the parameter server (private namespace: /albc_controller/)
rosparam load "$(rospack find albc_control)/config/albc_controller.yaml" /albc_controller

# Start joint_angle_command in background
rosrun albc_control joint_angle_command &
JAC_PID=$!

# Cleanup on exit: kill background node
cleanup() {
    kill "$JAC_PID" 2>/dev/null
    wait "$JAC_PID" 2>/dev/null
    exit
}
trap cleanup EXIT INT TERM

# Start albc_controller in foreground (stdin available for key input)
rosrun albc_control albc_controller __name:=albc_controller
