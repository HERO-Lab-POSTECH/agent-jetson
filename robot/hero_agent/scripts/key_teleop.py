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
═══════════════════════════════════════════════════
              HERO Key Teleop
═══════════════════════════════════════════════════
 STARTUP: e=Power y=YawON p=DepthON z=Spd wasd
═══════════════════════════════════════════════════
 Power   e/t=Relay  r/f=Laser  g=PWMInit
 Move    w/s/a/d  r/f=Heave
 Speed   z/x=+/-10  u/j=Throttle+/-10
 Yaw     y/h=ON/OFF  n=Reset  i/k=+/-0.1
 Depth   p/;=ON/OFF  o/l=+/-0.1
 Grip    c=Open  v=Stop  b=Close
──────────────── Jetson Only ──────────────────────
 Target  e=Send  q=Reset
 TDC     ,/.=ON/OFF  y/h=Mb  u=KKp  i=KKv
 Winch   1=Cal  2/3=Meter  4/5=Step
 Recov   z/x/c/v/b  /=ExpHold  ]=ExpClose
 Auto    t=Start  g=Stop
 Mosaic  p=On  o=Off   Dknet  n=On  m=Off
 Rec     [=Experiment  R=Rosbag
═══════════════════════════════════════════════════
 Ctrl+C to exit
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
