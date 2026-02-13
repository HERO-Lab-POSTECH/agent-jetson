#!/usr/bin/env python
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

HELP_TEXT = """
========== HERO Key Teleop ==========
 FSM:     0=Reset  -/+=Step
 TDC:     ,=On  .=Off
 Move:    w=Fwd  s=Back  a/d=Sway
 Target:  e=Send  q=Reset
 Control: y=Yaw  p=Depth
 Winch:   1=Calib  2/3=Meter  4/5=Step
 Recovery: z/x/c/v/b=Modes  t=Auto  g=Stop
 Mosaic:  p=Start  o=Stop
 Darknet: n=On  m=Off
 Depth:   r=Up  f=Down
 Ctrl+C to exit
======================================
"""

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
