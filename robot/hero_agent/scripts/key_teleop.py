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
 Move:     w/s=Surge  a/d=Sway  r/f=Heave
 Target:   e=Send(+RelayOFF)  q=Reset
 TDC:      ,=On  .=Off
 TDC Tune: y/h=Mb  u=KKp  i=KKv
 Winch:    1=Calib  2/3=Meter  4/5=Step
 Recovery: z=Approach x=Close c=Final v=Deploy
           b=Off  /=ExpHold  ]=ExpClose
 Auto:     t=Start(+RelayON)  g=Stop
 Mosaic:   p=Start  o=Stop
 Darknet:  n=On  m=Off
 Record:   [=Experiment  R=Rosbag
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
