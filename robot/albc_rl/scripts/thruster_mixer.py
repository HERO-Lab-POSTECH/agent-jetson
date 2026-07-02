#!/usr/bin/env python
"""ROS node: RL thruster mixer (agent-jetson UUV).

Bridges the RL policy's thruster output to the firmware ESC subscriber.

    rl_inference_node  --(/albc/thruster_cmd, Float32MultiArray[6])-->  THIS
        THIS  --(/hero_agent/thruster_pwm, hero_agent_thruster_cmd[6])-->  agent.ino

WHAT THIS NODE OWNS (and, deliberately, what it does NOT):
  * per-channel SIGN table (B1) ...... sim's thruster sign convention is NOT
        guaranteed to match the firmware's motor wiring (pid.cpp signs are
        non-uniform: +m1, -m2/-m4/-m5, -m0/-m3 for depth). The sign of each
        channel must be MEASURED in a restrained tank test, then filled into
        ~thruster_sign below. Until measured, all signs are +1 (identity) and
        the operator MUST keep thruster_scale tiny (0.05-0.1) while checking.
  * clamp to [-1, 1] ................. defensive; the policy contract is [-1,1]
        but a garbage/NaN upstream value must never reach the firmware mapping.
  * Float32MultiArray -> hero_agent_thruster_cmd conversion.

WHAT THIS NODE MUST NOT DO:
  * apply thruster_scale -- rl_inference_node ALREADY scales (rl_inference_node.py
        line ~352: action[2:8] * thruster_scale). Scaling again here would square
        the scale (0.1 -> 0.01) and silently break every intermediate tank-ramp
        step. Scale lives in ONE place: the RL node's ~thruster_scale param.
  * apply the PWM mapping -- that is the firmware's job (agent.ino
        rl_action_to_pwm), which also owns the narrow vertical span + DEPTH_BIAS.

The firmware has its own inter-message watchdog (B2) that NEUTRALs the ESCs if
this node dies, so a crash here fails safe.
"""
import rospy
from std_msgs.msg import Float32MultiArray
from hero_msgs.msg import hero_agent_thruster_cmd

NUM_THR = 6


class ThrusterMixer(object):
    def __init__(self):
        # B1 per-channel sign table [T0..T5]. Default identity (+1) until MEASURED.
        # Set via rosparam ~thruster_sign (list of 6 in {-1,+1}); a restrained
        # tank test drives each channel solo at low output and records observed
        # vs commanded direction. WRONG sign on a vertical channel (T0/T3) is an
        # uncommanded dive -- measure those first and most carefully.
        sign = rospy.get_param("~thruster_sign", [1, 1, 1, 1, 1, 1])
        if len(sign) != NUM_THR:
            rospy.logwarn("~thruster_sign has %d entries (need %d) -- using identity",
                          len(sign), NUM_THR)
            sign = [1] * NUM_THR
        self.sign = [1.0 if s >= 0 else -1.0 for s in sign]
        if self.sign == [1.0] * NUM_THR:
            rospy.logwarn("THRUSTER SIGN TABLE IS IDENTITY (unmeasured) -- keep "
                          "thruster_scale tiny (0.05-0.1) until B1 sign check is done")

        # queue_size=1 is DELIBERATE (freshest-command-wins): a thruster feed-
        # forward wants the newest command, never a stale backlog. A brief mixer
        # stall drops intermediate commands AND may trip the firmware's 300ms
        # watchdog to NEUTRAL -- both fail-safe. Do NOT enlarge the queue (that
        # reintroduces stale-command latency).
        self._pub = rospy.Publisher("/hero_agent/thruster_pwm",
                                    hero_agent_thruster_cmd, queue_size=1)
        rospy.Subscriber("/albc/thruster_cmd", Float32MultiArray,
                         self._on_cmd, queue_size=1)
        rospy.loginfo("thruster_mixer up: sign=%s  in /albc/thruster_cmd -> "
                      "out /hero_agent/thruster_pwm (NO scale here -- RL node owns scale)",
                      [int(s) for s in self.sign])

    def _on_cmd(self, msg):
        if len(msg.data) < NUM_THR:
            rospy.logwarn_throttle(2.0, "thruster_cmd has %d channels (< %d) -- dropping",
                                   len(msg.data), NUM_THR)
            return
        out = hero_agent_thruster_cmd()
        for i in range(NUM_THR):
            a = float(msg.data[i])
            # drop a non-finite value to 0 (safe neutral) rather than pass garbage
            if a != a or a in (float("inf"), float("-inf")):
                a = 0.0
            a *= self.sign[i]                 # B1 sign only -- NO scale
            a = max(-1.0, min(1.0, a))        # defensive clamp to policy contract
            out.thrust[i] = a
        self._pub.publish(out)


def main():
    rospy.init_node("thruster_mixer")
    ThrusterMixer()
    rospy.spin()


if __name__ == "__main__":
    main()
