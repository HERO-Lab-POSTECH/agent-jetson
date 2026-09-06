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

# --- RUN RECORD (2026-09-06) ------------------------------------------------
# THIS script, not albc.launch, is what the operator actually runs for TDC (it
# keeps albc_controller in the foreground so stdin still reaches the runtime
# keys). So until now EVERY TDC run went unrecorded: the RL path had been
# bagging since August and the baseline had nothing, which is fatal to a
# comparison whose whole purpose is the baseline column.
#
# Env, not flags, because this script takes no arguments today and the operator
# types it as a bare alias:
#   ALBC_RECORD=0                 skip the bag (bench check)
#   ALBC_SCENARIO=s1_attitude     scenario label -> /albc/run_meta
#   ALBC_NOTES="payload B, 400 g" operator note  -> /albc/run_meta
#   ALBC_CONTROLLER=tdc           override when this shell drives something else
#
# Backgrounded, and its failure is NOT fatal: `set -e` is on, and a recorder
# that cannot start must not stop the robot from coming up. A missing bag is a
# lost run; a robot that will not launch is a lost session.
REC_PID=""
if [ "${ALBC_RECORD:-1}" != "0" ]; then
    roslaunch albc_rl run_record.launch \
        controller:="${ALBC_CONTROLLER:-tdc}" \
        scenario:="${ALBC_SCENARIO:-}" \
        notes:="${ALBC_NOTES:-}" \
        bag_prefix:=tdc > /tmp/albc_run_record.log 2>&1 &
    REC_PID=$!
    echo "run record: PID $REC_PID (log /tmp/albc_run_record.log). ALBC_RECORD=0 to skip."
else
    echo "run record: SKIPPED (ALBC_RECORD=0). This run leaves no bag."
fi

# Cleanup on exit: kill background nodes.
# The recorder is stopped FIRST and with SIGINT, because rosbag writes its index
# only on a clean shutdown. A SIGKILLed recorder leaves a .bag.active that has
# to be reindexed before anything can read it.
cleanup() {
    # `|| true` on BOTH waits. `set -e` is on (line 12) and `wait` returns the
    # child's status: 128+signum for a signalled child, non-zero for a roslaunch
    # whose node died. Without it the shell exits INSIDE cleanup, before
    # `kill "$JAC_PID"` runs, and joint_angle_command survives as an orphan.
    # That is the most expensive fingerprint in this project: the next launch
    # starts a node of the same name, ROS kills the older one without an error,
    # and the dying one calls disableTorque() on the same physical motors.
    if [ -n "$REC_PID" ]; then
        kill -INT "$REC_PID" 2>/dev/null || true
        wait "$REC_PID" 2>/dev/null || true
    fi
    kill "$JAC_PID" 2>/dev/null || true
    wait "$JAC_PID" 2>/dev/null || true
    exit
}
trap cleanup EXIT INT TERM

# Start albc_controller in foreground (stdin available for key input)
rosrun albc_control albc_controller __name:=albc_controller
