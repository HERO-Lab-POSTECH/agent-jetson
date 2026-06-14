#!/usr/bin/env bash
# ros_lib 재생성 — vendor하지 않고 hero_msgs에서 생성 (drift 방지).
# 사용: bash regen_ros_lib.sh [dest]   (dest 기본 = ~/Arduino/libraries)
# -u 제외: ROS setup.bash가 unbound var(ROS_DISTRO 등)를 참조해 -u와 비호환.
set -eo pipefail
DEST="${1:-$HOME/Arduino/libraries}"

source /opt/ros/lunar/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"

# 1) hero_msgs 빌드 선행 (헤더가 최신이어야 ros_lib가 최신 필드 반영)
cd "$HOME/catkin_ws"
catkin_make --pkg hero_msgs

# 2) ros_lib 재생성
rm -rf "$DEST/ros_lib"
rosrun rosserial_arduino make_libraries.py "$DEST"

echo "ros_lib regenerated at $DEST/ros_lib"
ls "$DEST/ros_lib/hero_msgs" | head
