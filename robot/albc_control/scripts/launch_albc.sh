#!/bin/bash
# launch_albc.sh — Start ALBC controller (initial mode from control_mode param)
#
# Usage: launch-albc  (alias this script)
#
# The controller boots straight into the control_mode param (yaml, default
# 1=TDC) — there is no blocking startup mode prompt anymore. This script keeps
# albc_controller in the FOREGROUND so it still owns stdin for RUNTIME key input
# (= cycle, 1-4 select, MANUAL w/s/a/d/m). joint_angle_command runs in the
# background. (roslaunch albc.launch also works now; use this only for the keys.)

set -e

# ROS environment
source /opt/ros/lunar/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash

# Load parameters to the parameter server (private namespace: /albc_controller/)
rosparam load "$(rospack find albc_control)/config/albc_controller.yaml" /albc_controller

# Start joint_angle_command in background (suppress output to keep terminal clean)
rosrun albc_control joint_angle_command > /dev/null 2>&1 &
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
