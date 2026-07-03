#!/usr/bin/env python
"""ROS node: RL thruster mixer (agent-jetson UUV).

Bridges the RL policy's thruster output to the firmware ESC subscriber.

    rl_inference_node  --(/albc/thruster_cmd, Float32MultiArray[6])-->  THIS
        THIS  --(/hero_agent/thruster_pwm, hero_agent_thruster_cmd[6])-->  agent.ino

WHAT THIS NODE OWNS (and, deliberately, what it does NOT):
  * per-output-channel PERMUTATION (~thruster_order) -- the SIM numbers its
        thrusters differently from the firmware's ESC channels. The sim's TAM
        (constrained-albc config.py) puts the two VERTICAL (heave) thrusters at
        sim indices 4,5; the firmware wires its two vertical motors to channels
        m0,m3 (pid.cpp PID_control_depth drives only m0,m3). So a straight
        pass-through (identity) sends the policy's depth-hold thrust to the
        physically HORIZONTAL motors -> uncommanded dive. This node permutes sim
        channels into firmware-channel order so vertical-stays-vertical.
        ~thruster_order[j] = which SIM index feeds firmware output channel j.
        A startup axis-assertion forbids any order that crosses the axis split
        (that is the one mis-edit that dives the boat); the within-axis
        horizontal assignment is left editable because B1 must still measure it.
  * per-OUTPUT-channel SIGN table (B1) ...... sim's thruster sign convention is
        NOT guaranteed to match the firmware's motor wiring (pid.cpp signs are
        non-uniform). The sign of each PHYSICAL channel must be MEASURED in a
        restrained tank test, then filled into ~thruster_sign. sign is indexed
        by firmware OUTPUT channel (NOT sim channel): the operator drives output
        m_j solo and records "m0 pushed the wrong way -> flip sign[0]", and that
        record must stay valid even if ~thruster_order is later edited.
  * clamp to [-1, 1] ................. defensive; the policy contract is [-1,1]
        but a garbage/NaN upstream value must never reach the firmware mapping.
  * Float32MultiArray -> hero_agent_thruster_cmd conversion.

  ORDER OF OPERATIONS (per output channel j): permute -> sign -> clamp.
        out[j] = clamp( sign[j] * in[ order[j] ] )
        permute picks the source, sign corrects that physical channel, clamp is
        the last defensive gate.

WHAT THIS NODE MUST NOT DO:
  * apply thruster_scale -- rl_inference_node ALREADY scales (rl_inference_node.py
        line ~352: action[2:8] * thruster_scale). Scaling again here would square
        the scale (0.1 -> 0.01) and silently break every intermediate tank-ramp
        step. Scale lives in ONE place: the RL node's ~thruster_scale param.
  * apply the PWM mapping -- that is the firmware's job (agent.ino
        rl_action_to_pwm), which also owns the narrow vertical span + DEPTH_BIAS.
        The firmware applies that vertical span to m0,m3 BY CHANNEL INDEX -- which
        is correct ONLY once this node routes vertical sim commands into m0,m3.

The firmware has its own inter-message watchdog (B2) that NEUTRALs the ESCs if
this node dies, so a crash here fails safe.

TEMPORARY ADAPTER: this permutation is a deployment-side workaround for the
sim<->firmware channel-order mismatch. The permanent fix is to reorder the sim
TAM to match the firmware wiring and re-train; until then this node is the bridge.
"""
import math

import rospy
from std_msgs.msg import Float32MultiArray
from hero_msgs.msg import hero_agent_thruster_cmd

NUM_THR = 6

# Firmware channel axis sets (physical wiring, fixed):
#   vertical  = m0, m3   (pid.cpp PID_control_depth drives only these)
#   horizontal= m1,m2,m4,m5
FW_VERT_CH = (0, 3)
FW_HORZ_CH = (1, 2, 4, 5)
# Sim thruster axis sets (from constrained-albc TAM Fz row (0,0,0,0,1,1)):
#   vertical (heave) = sim indices 4,5 ; horizontal = 0,1,2,3
SIM_VERT = frozenset((4, 5))
SIM_HORZ = frozenset((0, 1, 2, 3))

# Safe default order: AXIS-correct, within-axis arbitrary (identity is KNOWN-WRONG
# -> dive). m0<-sim4, m3<-sim5 (both vertical); horizontal fw channels take sim
# {0,1,2,3}. The within-axis horizontal assignment + every sign are placeholders
# until B1 measures them; live safety is the RL node's thruster_scale=0.0 gate.
DEFAULT_ORDER = [4, 0, 1, 5, 2, 3]  # index = fw channel j, value = sim source


class ThrusterMixer(object):
    def __init__(self):
        order = rospy.get_param("~thruster_order", DEFAULT_ORDER)
        self.order = self._validate_order(order)

        # B1 per-OUTPUT-channel sign table [m0..m5]. Default identity (+1) until
        # MEASURED. Set via rosparam ~thruster_sign (list of 6 in {-1,+1}); a
        # restrained tank test drives each OUTPUT channel solo at low output and
        # records observed vs commanded direction. WRONG sign on a vertical
        # channel (m0/m3) is an uncommanded dive -- measure those first.
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
        rospy.loginfo("thruster_mixer up: order(fw<-sim)=%s sign=%s  "
                      "/albc/thruster_cmd -> /hero_agent/thruster_pwm "
                      "(NO scale here -- RL node owns scale)",
                      self.order, [int(s) for s in self.sign])

    def _validate_order(self, order):
        """Enforce the axis invariant: fw vertical channels (m0,m3) MUST source
        from sim vertical indices {4,5}, and fw horizontal channels from sim
        horizontal {0,1,2,3}. An axis-crossing order dives the boat, so we refuse
        to run rather than publish it. Within-axis assignment is NOT constrained
        (B1 measures it)."""
        ok = (isinstance(order, (list, tuple)) and len(order) == NUM_THR
              and sorted(int(x) for x in order) == list(range(NUM_THR)))
        if not ok:
            rospy.logfatal("~thruster_order %s is not a permutation of 0..5 -- "
                           "refusing to start (falling back would hide the misconfig)",
                           order)
            raise rospy.ROSInitException("invalid thruster_order")
        order = [int(x) for x in order]
        vert_src = set(order[c] for c in FW_VERT_CH)
        horz_src = set(order[c] for c in FW_HORZ_CH)
        if not (vert_src <= SIM_VERT and horz_src == SIM_HORZ):
            rospy.logfatal("~thruster_order %s CROSSES the axis split: fw vertical "
                           "channels m0,m3 must source sim vertical {4,5} (got %s) "
                           "and fw horizontal m1,m2,m4,m5 must source sim {0,1,2,3} "
                           "(got %s). This would route depth thrust to horizontal "
                           "motors -> uncommanded dive. Refusing to start.",
                           order, sorted(vert_src), sorted(horz_src))
            raise rospy.ROSInitException("thruster_order crosses axis split")
        return order

    def _on_cmd(self, msg):
        if len(msg.data) < NUM_THR:
            rospy.logwarn_throttle(2.0, "thruster_cmd has %d channels (< %d) -- dropping",
                                   len(msg.data), NUM_THR)
            return
        out = hero_agent_thruster_cmd()
        for j in range(NUM_THR):
            # permute: fw output channel j sources sim index order[j]
            a = float(msg.data[self.order[j]])
            # drop a non-finite value to 0 (safe neutral) rather than pass garbage.
            # NOTE: math.isfinite is py3-only; on ROS-lunar python2 use isnan/isinf.
            if math.isnan(a) or math.isinf(a):
                a = 0.0
            a *= self.sign[j]                 # sign per PHYSICAL channel -- NO scale
            a = max(-1.0, min(1.0, a))        # defensive clamp to policy contract
            out.thrust[j] = a
        self._pub.publish(out)


def main():
    rospy.init_node("thruster_mixer")
    ThrusterMixer()
    rospy.spin()


if __name__ == "__main__":
    main()
