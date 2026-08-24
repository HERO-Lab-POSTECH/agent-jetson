#!/bin/bash
# start_tdc_dryrun.sh — run albc_controller WITHOUT touching the arm.
#
# Usage: bash start_tdc_dryrun.sh [--check]
#
# Why this exists. albc_controller does not read the measured joint angles: it
# seeds its internal state from initial_theta{1,2}_deg and publishes that state
# straight out (albc_controller.cpp:209-215, publish :322). So when the yaml
# default (90/90) differs from where the arm actually is, the FIRST TICK is a
# jump command. On 2026-08-24 the arm sat at theta2 = 180.4 deg while the yaml
# said 90 -- launching for real would have driven J2 to pi/2, which is the
# "large force x maximum lever" pose that broke arm1 on 2026-08-22.
#
# PLAN section D-4 named this script as the way out and the file was lost with a
# board reboot (it only ever existed in /tmp). This is it, rebuilt.
#
# Two independent layers keep the arm still, either one sufficient:
#   1. The two joint command topics are REMAPPED to /tdc_dryrun/*, so the real
#      driver never sees them.
#   2. The script REFUSES to start while /joint_angle_command is alive. With no
#      driver running nothing subscribes to the real topics at all, so even a
#      botched remap cannot move anything. This is the layer that does not
#      depend on getting the remap right.
#
# Watch what the controller would have commanded:
#   rostopic echo /tdc_dryrun/joint1_cmd
#   rostopic echo /tdc_dryrun/joint2_cmd

set -e

J1_REAL="/hero_agent/active_joint1_position_controller/command"
J2_REAL="/hero_agent/active_joint2_position_controller/command"
J1_FAKE="/tdc_dryrun/joint1_cmd"
J2_FAKE="/tdc_dryrun/joint2_cmd"

source /opt/ros/lunar/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash

YAML="$(rospack find albc_control)/config/albc_controller.yaml"

# --- Layer 2: no driver, no motion ------------------------------------------
# Match the ROS node name, not the process, so a stale rosrun shell does not
# read as a live driver.
if rosnode list 2>/dev/null | grep -q '^/joint_angle_command$'; then
    echo "REFUSING: /joint_angle_command is running."
    echo "  A dry run must have no subscriber on the real command topics."
    echo "  Stop the driver (or launch-albc) first, then rerun."
    exit 1
fi

if [ "$1" = "--check" ]; then
    echo "== start_tdc_dryrun --check =="
    echo "  yaml          : $YAML"
    if [ -f "$YAML" ]; then
        echo "  yaml exists   : yes"
    else
        echo "  yaml exists   : NO"
        exit 1
    fi
    echo "  seed theta1   : $(grep '^initial_theta1_deg' "$YAML")"
    echo "  seed theta2   : $(grep '^initial_theta2_deg' "$YAML")"
    echo "  control_mode  : $(grep '^control_mode' "$YAML")"
    echo "  driver alive  : no  (checked above, else we would have exited)"
    echo "  remap         : $J1_REAL -> $J1_FAKE"
    echo "                  $J2_REAL -> $J2_FAKE"
    echo "  would run     : rosrun albc_control albc_controller __name:=albc_controller_dryrun \\"
    echo "                    $J1_REAL:=$J1_FAKE $J2_REAL:=$J2_FAKE"
    echo "OK -- nothing was started."
    exit 0
fi

rosparam load "$YAML" /albc_controller_dryrun

echo "== TDC DRY RUN -- the arm will not move =="
echo "   joint commands go to $J1_FAKE / $J2_FAKE"
echo "   keys work as usual (cycle, 1-4 mode select, manual w/s/a/d/m)"
echo ""

# --- Layer 1: remap the outputs. Foreground so stdin still drives the keys. ---
# _allow_yaml_seed: this tool runs with NO driver (it refuses to start when one
# is alive), so /albc/joint_states is absent BY DESIGN and the controller would
# otherwise refuse to seed. The bench seed sweep is the whole point of this tool,
# and the joint outputs are remapped below, so the yaml seed cannot reach the arm.
exec rosrun albc_control albc_controller __name:=albc_controller_dryrun \
    _allow_yaml_seed:=true \
    "$J1_REAL:=$J1_FAKE" \
    "$J2_REAL:=$J2_FAKE"
