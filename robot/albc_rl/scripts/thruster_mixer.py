#!/usr/bin/env python
"""ROS node: RL thruster mixer (agent-jetson UUV).

Bridges the RL policy's thruster output to the firmware ESC subscriber.

    rl_inference_node  --(/albc/thruster_cmd, Float32MultiArray[6])-->  THIS
        THIS  --(/hero_agent/thruster_pwm, hero_agent_thruster_cmd[6])-->  agent.ino

WHAT THIS NODE OWNS (and, deliberately, what it does NOT):
  * per-output-channel PERMUTATION (~thruster_order) -- the SIM numbers its
        thrusters differently from the firmware's ESC channels. In the DEPLOYED
        matrix the two vertical columns are 0 and 3 (Fz row = 1,0,0,1,0,0), which
        happens to match the firmware's vertical channels m0,m3 by index -- but
        NOT by position: sim col0 sits at 9h and col3 at 3h, while the measured
        m0 is at 3h and m3 at 9h, so the vertical pair is swapped and identity
        would invert every depth correction. The four horizontals are rotated
        90 deg besides. This node permutes sim channels into firmware-channel
        order so vertical-stays-vertical AND each corner gets its own column.
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

  * FAULT REALLOCATION, opt-in via ~fault_reallocate (DEFAULT OFF, 2026-08-24).
        A disabled channel (~thruster_sign[j] = 0) is otherwise only MUTED: the
        policy's intended wrench then loses that column, so the response does not
        merely weaken, it points elsewhere. With this on, the node reads the
        intended wrench off the DEPLOYED matrix (deployed_tam.json) and re-solves
        it onto the live channels -- the RL-side counterpart of what pid.cpp
        3771674 already does for the classic path. See reallocate(). It is off by
        default because it changes deployed behaviour and needs a tank re-check.

  ORDER OF OPERATIONS (per output channel j): permute -> sign -> deadband -> clamp.
        With ~fault_reallocate on, REALLOCATE REPLACES PERMUTE (it consumes
        `order` itself and returns fw-indexed values); the other three stages are
        identical, so the two paths differ in exactly one step.
        out[j] = clamp( undeadband( sign[j] * in[ order[j] ] ) )
        permute picks the source, sign corrects that physical channel (0 = DISABLED,
        pinned to exact neutral -- see normalize_sign), undeadband
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
        rl_action_to_pwm). As of 2026-08-12 all SIX RL channels share one mapping
        (span 300, ESC_MIN/MAX, bias 0), so a zero command is exactly ESC_NEUTRAL
        on every channel -- the same value the B2 watchdog falls back to. The old
        narrow vertical span (150) + DEPTH_BIAS (30) is GONE from the RL path; the
        classic depth PID (pid.cpp) still uses DEPTH_BIAS and was not touched.
        Because the mapping is now uniform, it no longer depends on this node
        routing vertical commands into m0,m3 -- but the routing still must be
        right, or depth thrust lands on horizontal motors.

The firmware has its own inter-message watchdog (B2) that NEUTRALs the ESCs if
this node dies, so a crash here fails safe.

TEMPORARY ADAPTER: this permutation is a deployment-side workaround for the
sim<->firmware channel-order mismatch. The permanent fix is to reorder the sim
TAM to match the firmware wiring and re-train; until then this node is the bridge.
"""
import json
import math
import os

import rospy
from std_msgs.msg import Float32MultiArray
from hero_msgs.msg import hero_agent_thruster_cmd
from albc_rl.contract import TOPICS

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

# Channel order, sign set, and the deadband inverse are DERIVED, not chosen --
# deployed_tam.json is the authority and test_deploy_constants.py re-derives
# them on every run. Do NOT edit the constants below to make a bag look right.
# Four wrong orders shipped before [3,2,4,0,5,1] stuck; the full derivation,
# the measured channel map, and what each sign means physically:
#   docs/adr/003-thruster-order-and-sign.md
DEFAULT_ORDER = [3, 2, 4, 0, 5, 1]  # index = fw channel j, value = sim source

# ESC deadband, normalized to the action range, for the 2026-08-12 firmware.
DEFAULT_DEADBAND = 0.15


def normalize_sign(sign):
    """Per-OUTPUT-channel sign table -> floats. 0 means DISABLED, not +1.

    +1/-1 pick the rotation direction of that physical channel. 0 pins the channel
    to exactly neutral: `a *= 0.0` lands in undeadband()'s |a| < 1e-3 branch, which
    returns 0.0, so the ESC sees dead centre rather than a deadband-lifted value.

    Why a disable exists at all (2026-08-24): m4 has an intermittent open-phase
    fault -- it bites at random rather than being reliably dead, and it cannot be
    unplugged on this vehicle. An INTERMITTENT channel is worse for the deployed
    policy than a dead one: fault DR trained effectively-dead channels at ~0.5%
    per episode (PLAN 0i-3) and never trained a channel that comes and goes, and
    `use_privileged_fault_obs: false` means the policy cannot see which channel
    misbehaved. Pinning the channel makes the real fault match the one the policy
    has actually seen.

    Guard: `s >= 0` used to map 0 to +1, which silently ARMED a channel someone
    had tried to switch off. Keep the `s == 0` branch first.

    Module-level and pure so it is testable without rospy (see
    test_thruster_mixer_axes.py, which parses this file rather than importing it).
    """
    return [0.0 if s == 0 else (1.0 if s > 0 else -1.0) for s in sign]


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


def solve3x3(a, b):
    """Cramer's rule. `a` is 3 rows of 3, `b` is 3. None if singular.

    Stdlib only on purpose -- this runs on the board's python2.7 inside a 50 Hz
    callback, and pulling numpy in for one 3x3 would be the heaviest thing in
    the node. Cramer is fine at this size and has no pivoting to get wrong.
    """
    def det(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))

    d = det(a)
    if abs(d) < 1e-9:
        return None
    out = []
    for c in range(3):
        m = [[b[r] if k == c else a[r][k] for k in range(3)] for r in range(3)]
        out.append(det(m) / d)
    return out


def reallocate(action, order, sign, alloc):
    """Re-solve the policy's INTENDED wrench onto the channels that still work.

    WHAT PROBLEM THIS SOLVES
    ------------------------
    `normalize_sign`'s 0 only TURNS A CHANNEL OFF. The policy asked for a wrench
    and got that wrench minus one column, so a dead m4 does not just weaken the
    response -- it points it somewhere else. The classic firmware path already
    fixed this on its side (pid.cpp 3771674 drops one channel per mode and the
    remaining two reproduce the heading exactly); this is the same idea for RL.

    WHY THIS IS NOT "the policy is now out of distribution"
    ------------------------------------------------------
    The policy learned action <-> wrench on the SIM TAM. Reallocation makes the
    realised wrench match that learned relation again, so it moves the plant
    TOWARDS the training distribution, not away. This is also why the answer to
    "do we retrain" is no: fault DR trained a single effectively-dead channel at
    ~0.5% per episode, two simultaneously at 1/30,000, and
    `use_privileged_fault_obs: false` means the policy cannot even see which
    channel died. That coverage is too thin to lean on -- deployment-side
    reallocation restores the plant the policy already knows.

    WHAT IT CAN AND CANNOT RECOVER
    ------------------------------
    Horizontal: three live channels, three axes (Fx, Fy, Mz) -> regular 3x3
        (det 0.288, cond 7.02 on the deployed matrix). Direction is recovered
        EXACTLY; the cost is magnitude, capped near 50% of the 4-channel reach.
    Vertical:   m3 is dead, so (Fz, My) collapses to rank 1. Fz and My CANNOT be
        separated -- one has to be abandoned. Fz wins here: depth is the control
        objective and attitude is the arm's job in the ALBC design. Every heave
        command therefore carries My = -0.145 per unit Fz as an unavoidable
        pitch disturbance. Say so out loud rather than pretending it is solved.

    ARGUMENTS
    ---------
    action : 6 policy values in [-1,1], SIM action index order (msg.data as-is).
    order  : ~thruster_order. order[j] = which sim index feeds fw channel j.
    sign   : normalised sign table. 0 marks a channel DEAD (see normalize_sign).
    alloc  : allocation_matrix from deployed_tam.json -- the DEPLOYED checkpoint's
             own matrix. Read the artifact, never a source-tree constant: that
             single habit is what four wrong thruster_order values came from.

    RETURNS a list of 6 SIM-FRAME values already indexed by FIRMWARE channel, so
    the caller must NOT apply `order` again -- sign, undeadband and clamp still
    run exactly as they do on the plain path.

    Module-level and pure so it is testable without rospy (see
    test_thruster_reallocation.py, which parses this file rather than importing).
    """
    fx, fy, fz, mz = alloc["Fx"], alloc["Fy"], alloc["Fz"], alloc["Mz"]

    a = []
    for i in range(NUM_THR):
        v = float(action[i])
        a.append(0.0 if (math.isnan(v) or math.isinf(v)) else v)

    # The wrench the policy MEANT, over all six sim columns.
    w_fx = sum(fx[i] * a[i] for i in range(NUM_THR))
    w_fy = sum(fy[i] * a[i] for i in range(NUM_THR))
    w_fz = sum(fz[i] * a[i] for i in range(NUM_THR))
    w_mz = sum(mz[i] * a[i] for i in range(NUM_THR))

    out = [0.0] * NUM_THR
    horz = [j for j in FW_HORZ_CH if sign[j] != 0.0]
    vert = [j for j in FW_VERT_CH if sign[j] != 0.0]

    # --- horizontal ---------------------------------------------------------
    if len(horz) == len(FW_HORZ_CH):
        for j in FW_HORZ_CH:          # nothing is broken; do not disturb anything
            out[j] = a[order[j]]
    elif len(horz) == 3:
        cols = [order[j] for j in horz]
        v = solve3x3([[fx[c] for c in cols],
                      [fy[c] for c in cols],
                      [mz[c] for c in cols]], [w_fx, w_fy, w_mz])
        if v is None:                 # singular: no 3-channel solution exists
            for j in FW_HORZ_CH:
                out[j] = a[order[j]] if sign[j] != 0.0 else 0.0
        else:
            for k, j in enumerate(horz):
                out[j] = v[k]
    else:
        # 2 or fewer live horizontals: (Fx,Fy,Mz) is rank-deficient and there is
        # no honest re-solve. Pass through and let the caller's warning stand.
        for j in FW_HORZ_CH:
            out[j] = a[order[j]] if sign[j] != 0.0 else 0.0

    # --- vertical -----------------------------------------------------------
    if len(vert) == len(FW_VERT_CH):
        for j in FW_VERT_CH:
            out[j] = a[order[j]]
    elif len(vert) == 1:
        j = vert[0]
        c = order[j]
        if fz[c] != 0.0:
            out[j] = w_fz / fz[c]     # My is abandoned -- see the docstring

    # --- saturation ---------------------------------------------------------
    # Scale the whole GROUP down, never clamp a single channel: clamping one
    # channel of a solved set rotates the realised wrench, which is the exact
    # failure reallocation exists to prevent. The two groups scale independently
    # because they no longer share an axis (My is already abandoned), so a
    # saturated turn must not throttle depth.
    for group in (horz, vert):
        peak = 0.0
        for j in group:
            peak = max(peak, abs(out[j]))
        if peak > 1.0:
            for j in group:
                out[j] /= peak
    return out


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
        self.sign = normalize_sign(sign)
        for j, sj in enumerate(self.sign):
            if sj == 0.0:
                rospy.logwarn("thruster m%d is DISABLED (~thruster_sign[%d]=0) -- "
                              "it will be held at exact neutral", j, j)

        # Fault-tolerant reallocation. DEFAULT OFF -- it changes what the ESCs
        # receive for the same policy output, so it is a deployment behaviour
        # change and needs an operator decision plus a tank re-verification, not
        # a quiet default flip. Turn on with ~fault_reallocate:=true once that
        # has happened. See reallocate() for what it does and does not recover.
        self.alloc = None
        if bool(rospy.get_param("~fault_reallocate", False)):
            self.alloc = self._load_alloc()
        rospy.loginfo("fault reallocation: %s",
                      "ON" if self.alloc else "off (channels are only muted, not redistributed)")

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
        self._pub = rospy.Publisher(TOPICS["thruster_pwm"],
                                    hero_agent_thruster_cmd, queue_size=1)
        rospy.Subscriber(TOPICS["thruster_cmd"], Float32MultiArray,
                         self._on_cmd, queue_size=1)
        rospy.loginfo("thruster_mixer up: order(fw<-sim)=%s sign=%s  "
                      "/albc/thruster_cmd -> /hero_agent/thruster_pwm "
                      "(NO scale here -- RL node owns scale)",
                      self.order, [int(s) for s in self.sign])

    def _load_alloc(self):
        """The DEPLOYED checkpoint's own allocation matrix, from the artifact.

        Refuses to start rather than falling back: a mixer that silently runs
        WITHOUT reallocation after being asked for it looks identical to one that
        is working, and that silent-success class is exactly how four wrong
        thruster_order values survived for weeks.

        Also cross-checks the sign table against the artifact's measured channel
        health. Marking only m4 dead while m3 is recorded DEAD would leave the
        vertical solve splitting Fz across a motor that does not turn -- half the
        heave, no error anywhere.
        """
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "deployed_tam.json")
        try:
            with open(path) as f:
                tam = json.load(f)
            alloc = tam["allocation_matrix"]
            for row in ("Fx", "Fy", "Fz", "Mz"):
                if len(alloc[row]) != NUM_THR:
                    raise ValueError("allocation_matrix.%s is not %d wide" % (row, NUM_THR))
        except Exception as exc:       # noqa: BLE001 -- refuse loudly, whatever broke
            rospy.logfatal("~fault_reallocate is set but %s is unusable (%s) -- "
                           "refusing to start rather than silently running without "
                           "reallocation", path, exc)
            raise rospy.ROSInitException("deployed_tam.json unusable")

        health = tam.get("measured_channel_map", {})
        for j in range(NUM_THR):
            status = health.get("m%d" % j, {}).get("status", "ok")
            if status != "ok" and self.sign[j] != 0.0:
                rospy.logwarn("m%d is recorded %s in deployed_tam.json but "
                              "~thruster_sign[%d] still ENABLES it -- reallocation "
                              "will hand it thrust it cannot deliver", j, status, j)
        return alloc

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
            # NOTE the sets below are the LIVE-TAM indices (SIM_VERT / SIM_HORZ), not
            # the pre-reorder _BASE_ALLOCATION_MATRIX ones. This message said {4,5} /
            # {0,1,2,3} until 2026-08-12 -- stale from the old convention, and the one
            # string an operator reads at the exact moment the node refuses to start.
            rospy.logfatal("~thruster_order %s CROSSES the axis split: fw vertical "
                           "channels m0,m3 must source sim vertical {0,3} (got %s) "
                           "and fw horizontal m1,m2,m4,m5 must source sim {1,2,4,5} "
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
        # Reallocation REPLACES the permute step (it returns fw-indexed values and
        # has already consumed `order`); sign, undeadband and clamp are unchanged
        # either way, so the two paths differ in exactly one line below.
        realloc = (reallocate(msg.data, self.order, self.sign, self.alloc)
                   if self.alloc else None)
        for j in range(NUM_THR):
            if realloc is not None:
                a = realloc[j]
            else:
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
