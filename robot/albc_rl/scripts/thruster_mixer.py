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

  ORDER OF OPERATIONS (per output channel j): permute -> sign -> deadband -> clamp.
        out[j] = clamp( undeadband( sign[j] * in[ order[j] ] ) )
        permute picks the source, sign corrects that physical channel, undeadband
        inverts the ESC's dead zone (see _undeadband), clamp is the last defensive
        gate. Deadband comes AFTER sign because the dead zone is a property of the
        physical channel, so it must act on the value that channel will actually
        receive -- inverting before the sign flip would push the wrong direction.

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
# Sim ACTION axis sets. The policy's action[2:8] is ALREADY in firmware channel
# order: constrained-albc builds its live TAM as
#     allocation_matrix = _reorder_columns(_BASE_ALLOCATION_MATRIX, _ESC_CHANNEL_ORDER)
# (envs/main/config.py), so the reorder happens sim-side. The LIVE Fz row recorded
# in the deployed teacher run (logs/.../trpo_iterbudget_s30_260805_012813/
# params/env.yaml) is (1,0,0,1,0,0) -- vertical at columns 0 and 3, exactly the
# firmware wiring. The pre-reorder (0,0,0,0,1,1) that the old {4,5} came from is
# _BASE_ALLOCATION_MATRIX, which the board never sees.
SIM_VERT = frozenset((0, 3))
SIM_HORZ = frozenset((1, 2, 4, 5))

# Default order MEASURED 2026-08-11 (dry, one channel at a time via
# b1_channel_probe, gripper at 12 o'clock on the observer's clock face):
#     m0 = 3h vertical    m1 = 1.5h    m2 = 4.5h
#     m3 = 9h vertical    m4 = 7.5h    m5 = 10.5h        (m3's motor is DEAD)
# Sim side from actuators.xacro: T0..T3 at the four diagonals (+-0.102, +-0.102),
# T4/T5 on +-x at 0.1445 m. Those coordinates reproduce the TAM's Mz=+-0.144 and
# My=+-0.145 to four decimals, so the sim geometry is exact, and with +x=3h,
# +y=12h (agent.urdf puts the gripper at +y) the LIVE columns land at
#     col0=T4 9h    col1=T0 7.5h   col2=T1 10.5h
#     col3=T5 3h    col4=T2 4.5h   col5=T3 1.5h
# (col_k = T[_ESC_CHANNEL_ORDER[k]] with the tuple (4,0,1,5,2,3) -- recompute this
#  block from the tuple, never from memory; a mis-recalled tuple is what produced
#  the previous wrong default.)
# Matching fw channel to live column BY PHYSICAL POSITION gives the order below.
#
# THREE earlier defaults were wrong; this is the fourth. [4,0,1,5,2,3] applied the
# sim's own column reorder a SECOND time and routed horizontal commands into the
# vertical motors (an uncommanded dive the axis assert could not catch while
# SIM_VERT was the stale {4,5}). Identity assumed the sim-side _ESC_CHANNEL_ORDER
# already matched the wiring. [3,2,4,0,5,1] (2026-08-11) had the right METHOD but
# read _ESC_CHANNEL_ORDER as (4,1,3,5,2,0) -- a value that is not in the repo. The
# live tuple is (4,0,1,5,2,3), introduced 2026-07-03 (constrained-albc 238932c) and
# UNCHANGED since; every later config.py commit is DR/latency work. The deployed
# teacher trained 2026-08-05, so that tuple is what the policy learned. Verified by
# git log, not by reading the working tree -- the working tree cannot tell you what
# a checkpoint was trained with.
# None of the wrong orders was ever exercised in the water: thruster_scale defaults
# to 0.0 and the field-test bags (fieldtest_2026-07-06-*) carry thruster_pwm == 0.
#
# Per-channel SIGNS remain placeholders until the tank measurement. NOTE the tank
# YAW test cannot validate this order: Mz = +0.144 for ALL FOUR horizontals, so any
# permutation of them yaws identically. Only Fx/Fy (translation direction) separates
# them, which is why the order is fixed from geometry and the water only confirms.
DEFAULT_ORDER = [3, 5, 4, 0, 1, 2]  # index = fw channel j, value = sim source

# ESC deadband, normalized to the action range, for the 2026-08-12 firmware.
DEFAULT_DEADBAND = 0.15


def undeadband(a, deadband):
    """Map a policy action onto the ESC's LIVE range, skipping the deadband.

    MEASURED 2026-08-12 (dry, one channel at a time): an ESC does not turn until
    the pulse leaves 1450..1545 us, i.e. about +-48 us around neutral. At span 300
    that is a normalized |a| < 0.15 producing NO thrust, while the policy learned a
    linear plant through zero. Two channels with different span AND bias (m2
    horizontal, m0 vertical) both broke away at exactly 1545 us -- that agreement
    across two different mappings is what pins the number.

    Inverse: to get the sim's thrust fraction t, command D + (1-D)*t, so a=0 -> 0
    (exactly neutral, no creep) and a=+-1 -> +-1 (full authority preserved).
    ponytail: assumes thrust ramps linearly straight out of the deadband; a real
    ESC steps a little at breakaway. Measuring that step needs a thrust stand we do
    not have -- revisit only if the tank shows a jump at low command.

    PRESUPPOSES the 2026-08-12 firmware (all six channels span 300, bias 0). On the
    OLD firmware the verticals sat at 1470 with span 150, so their deadband was
    asymmetric (+0.50 / -0.13) and one scalar cannot express it -- run with
    ~thruster_deadband:=0.0 until the board is reflashed.

    Module-level and pure so it is testable without rospy (see
    test_thruster_mixer_axes.py, which parses this file rather than importing it).
    """
    if abs(a) < 1e-3:
        return 0.0
    if deadband <= 0.0:
        return a
    s = 1.0 if a > 0.0 else -1.0
    return s * (deadband + (1.0 - deadband) * abs(a))


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

        # ESC deadband, normalized to the action range. See undeadband().
        # 0.0 disables the compensation (use that on pre-2026-08-12 firmware).
        self.deadband = float(rospy.get_param("~thruster_deadband", DEFAULT_DEADBAND))
        if not (0.0 <= self.deadband < 0.9):
            rospy.logwarn("~thruster_deadband %.3f out of [0,0.9) -- disabling",
                          self.deadband)
            self.deadband = 0.0
        rospy.loginfo("deadband compensation: %.3f%s", self.deadband,
                      "" if self.deadband > 0.0 else " (DISABLED)")
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
        from sim vertical action indices {0,3}, and fw horizontal channels from
        sim horizontal {1,2,4,5}. An axis-crossing order dives the boat, so we
        refuse to run rather than publish it. Within-axis assignment is NOT
        constrained (B1 measures it).

        The sim index sets here are LIVE-TAM indices (post _ESC_CHANNEL_ORDER),
        not _BASE_ALLOCATION_MATRIX indices -- see the SIM_VERT comment. Getting
        that wrong is what let the double-permutation default through."""
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
            a = undeadband(a, self.deadband)  # invert the ESC deadband (plant inverse)
            a = max(-1.0, min(1.0, a))        # defensive clamp to policy contract
            out.thrust[j] = a
        self._pub.publish(out)


def main():
    rospy.init_node("thruster_mixer")
    ThrusterMixer()
    rospy.spin()


if __name__ == "__main__":
    main()
