#!/usr/bin/env python
"""B1 thruster channel probe (agent-jetson UUV) -- restrained-tank bring-up tool.

Publishes DIRECTLY to /hero_agent/thruster_pwm, BYPASSING the mixer, so that
thrust[j] maps 1:1 to firmware ESC channel m_j. This makes solo-channel
attribution unambiguous: you command channel j and watch physical motor m_j.
(Going through the mixer would permute the channel via ~thruster_order, which is
exactly what B1 is measuring -- so we must not use it here.)

WHAT B1 MEASURES (fill these into albc_rl_fieldtest.launch after):
  * ~thruster_order : which sim horizontal index -> which fw horizontal channel
  * ~thruster_sign  : per-fw-channel rotation direction vs commanded sign

SAFETY (read before running):
  * The RELAY must be ON for the ESCs to have power (launch-agent, key 1). With
    relay OFF this script publishes but nothing spins -- safe dry run.
  * Drive ONE channel at a time at LOW output (default 0.05). Do vertical
    channels m0, m3 FIRST and most carefully -- a wrong vertical sign is a dive.
  * The firmware B2 watchdog NEUTRALs all channels if commands stop for >300ms,
    so this script publishes at 20 Hz for a fixed DURATION then sends an explicit
    NEUTRAL (all zeros) and exits. Ctrl-C also sends NEUTRAL on the way out.
  * This tool is a bring-up probe, NOT part of the live pipeline. Do not launch
    it alongside the mixer (two publishers on one topic = garbage).

USAGE (on the board, after `source devel/setup.bash`):
    rosrun albc_rl b1_channel_probe.py _channel:=0 _level:=0.05 _duration:=3.0
  or drive with a specific sign to check direction:
    rosrun albc_rl b1_channel_probe.py _channel:=0 _level:=-0.05 _duration:=3.0
  all-neutral (panic stop / verify zero):
    rosrun albc_rl b1_channel_probe.py _channel:=-1
"""
import rospy
from hero_msgs.msg import hero_agent_thruster_cmd

NUM_THR = 6
RATE_HZ = 20.0   # >3.3 Hz so the firmware 300ms watchdog stays armed


def build_msg(channel, level):
    m = hero_agent_thruster_cmd()
    for i in range(NUM_THR):
        m.thrust[i] = 0.0
    if 0 <= channel < NUM_THR:
        # Clamp to the policy contract [-1, 1] only. The earlier bring-up ceiling
        # (0.3, then 0.5) was removed at the operator's request on 2026-07-05:
        # vertical m0/m3 need more than 0.3 to clear DEPTH_BIAS=30 plus the narrow
        # RL_PWM_SPAN_VERT=150, and the operator drives and judges risk directly.
        # The firmware still constrains PWM to ESC_MIN/MAX and DEPTH_PWM_MIN/MAX,
        # so hardware limits stay enforced downstream regardless of this value.
        # WARNING: near +-1 on a FREE (untethered) robot this is FULL thrust and
        # it moves fast / dives; the operator stops it if it gets risky.
        lvl = max(-1.0, min(1.0, float(level)))
        m.thrust[channel] = lvl
    return m


def main():
    rospy.init_node("b1_channel_probe")
    channel = int(rospy.get_param("~channel", -1))       # -1 = all neutral
    level = float(rospy.get_param("~level", 0.05))
    duration = float(rospy.get_param("~duration", 3.0))

    pub = rospy.Publisher("/hero_agent/thruster_pwm",
                          hero_agent_thruster_cmd, queue_size=1)

    neutral = build_msg(-1, 0.0)

    def send_neutral(*_):
        # best-effort explicit stop (the 300ms watchdog is the real backstop)
        for _i in range(5):
            pub.publish(neutral)
            rospy.sleep(0.02)

    rospy.on_shutdown(send_neutral)

    # let the publisher connection establish before the timed drive
    rospy.sleep(0.5)

    if not (0 <= channel < NUM_THR):
        rospy.loginfo("b1_channel_probe: ALL-NEUTRAL (channel=%d) -- sending zeros", channel)
        send_neutral()
        return

    msg = build_msg(channel, level)
    rospy.logwarn("b1_channel_probe: driving m%d at %.3f for %.1fs "
                  "(watch physical motor m%d: is it VERTICAL/HORIZONTAL? which way?)",
                  channel, msg.thrust[channel], duration, channel)

    rate = rospy.Rate(RATE_HZ)
    t_end = rospy.Time.now() + rospy.Duration.from_sec(duration)
    while not rospy.is_shutdown() and rospy.Time.now() < t_end:
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("b1_channel_probe: duration elapsed -> NEUTRAL")
    send_neutral()


if __name__ == "__main__":
    main()
