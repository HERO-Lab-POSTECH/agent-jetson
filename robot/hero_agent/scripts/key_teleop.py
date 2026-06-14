#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Keyboard teleop node for HERO Agent.

Reads keyboard input and publishes to /hero_agent/key_input topic.
Run in a separate terminal for keyboard control while launch-agent runs.

Usage:
    rosrun hero_agent key_teleop.py
"""
import rospy
from std_msgs.msg import Int8
import sys
import tty
import termios
import os
import signal

# keymap.h KEYMAP[]와 lock-step (사용자키, 동작). 변경 시 양쪽 동시 수정.
# 'R'(CSV log)은 keymap 범위 밖 agent 내부 플래그(csv_logger.toggle) — 광고만 여기.
KEY_TABLE = [
    ("System",   [("1","Relay(toggle)"),("2","PWM Neutral"),("N","Yaw Reset"),("R","CSV log(toggle)")]),
    ("Control",  [("3","Yaw Ctrl(toggle)"),("4","Depth Ctrl(toggle)"),("5","Laser(toggle)")]),
    ("Jog",      [("w/s","Fwd/Back"),("a/d","Left/Right"),("q","STOP")]),
    ("Speed",    [("y/h","Speed +/-")]),
    ("Throttle", [("u/j","Throttle +/-")]),
    ("Setpoint", [("i/k","Yaw +/-0.1"),("o/l","Depth +/-0.1")]),
    ("Gripper",  [("c/v/b","Open/Stop/Close")]),
    ("Heave",    [("r/f","Up/Down")]),
]

def build_help():
    lines = ["", "="*51, "        HERO Key Teleop (V4 allow-list)", "="*51]
    for grp, keys in KEY_TABLE:
        body = "  ".join("%s=%s" % (k, d) for k, d in keys)
        lines.append(" %-9s %s" % (grp, body))
    lines.append("-"*51)
    lines.append(" Unlisted keys are ignored.  Ctrl+C to exit")
    lines.append("="*51)
    return "\n".join(lines)

HELP_TEXT = build_help()

running = True

def signal_handler(sig, frame):
    global running
    running = False

def main():
    global running
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('key_teleop', anonymous=True, disable_signals=True)
    pub = rospy.Publisher('/hero_agent/key_input', Int8, queue_size=10)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    print(HELP_TEXT)
    rospy.loginfo("Key teleop started. Press keys to send commands.")

    try:
        while running and not rospy.is_shutdown():
            try:
                ch = os.read(fd, 1)
            except OSError:
                continue
            if not ch:
                continue
            c = ord(ch)
            if c == 3:  # Ctrl+C
                break
            if c > 127:  # Int8 range: -128~127, skip non-ASCII keys
                continue
            pub.publish(Int8(data=c))
            rospy.loginfo("Key: '%s' (%d)", ch, c)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nTeleop stopped.")

if __name__ == '__main__':
    main()
