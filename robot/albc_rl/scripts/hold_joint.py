#!/usr/bin/env python
"""Hold ONE joint at a fixed absolute angle (bring-up helper).

Publishes a constant std_msgs/Float64 to a joint position-controller command
topic at a fixed rate, so the Dynamixel driver keeps torque on that target.
Replaces the `rostopic pub -r ... 'data: X'` idiom, which does not survive
roslaunch's args splitter (the hold nodes silently died on launch, the arm went
slack, and nothing published to the command topic -- observed 2026-07-05).

Params (private):
  ~topic  : full command topic (std_msgs/Float64)
  ~angle  : absolute angle in rad to hold
  ~rate   : publish rate in Hz (default 10)

The command is an ABSOLUTE angle, so republishing the same value never drifts --
the driver just keeps the target where it is.
"""
import rospy
from std_msgs.msg import Float64
from albc_rl.contract import TOPICS


def main():
    rospy.init_node("hold_joint")
    topic = rospy.get_param("~topic", TOPICS["joint1_cmd"])
    angle = float(rospy.get_param("~angle"))
    rate_hz = float(rospy.get_param("~rate", 10.0))

    pub = rospy.Publisher(topic, Float64, queue_size=1)
    rospy.loginfo("hold_joint: holding %s at %.5f rad @ %.1f Hz", topic, angle, rate_hz)

    msg = Float64()
    msg.data = angle
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
