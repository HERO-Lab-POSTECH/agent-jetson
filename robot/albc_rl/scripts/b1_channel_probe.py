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
  * PARAMS ARE ONE-SHOT: `rosrun _p:=v` params PERSIST on the ROS param server
    after the node exits, so each run DELETEs its own params on startup (after
    reading them). Pass EVERY flag you need on EVERY run -- nothing carries over.
    (Before this, a leftover ~pair silently overrode later runs incl. the panic
    stop -- observed 2026-07-05.) An explicit `_channel:=-1` panic is checked
    FIRST and can never be overridden by any leftover mode param.

USAGE (on the board, after `source devel/setup.bash`):
    rosrun albc_rl b1_channel_probe.py _channel:=0 _level:=0.05 _duration:=3.0
  or drive with a specific sign to check direction:
    rosrun albc_rl b1_channel_probe.py _channel:=0 _level:=-0.05 _duration:=3.0
  all-neutral (panic stop / verify zero):
    rosrun albc_rl b1_channel_probe.py _channel:=-1
  multi-channel simultaneous drive (e.g. horizontal pair m1+m4, same level):
    rosrun albc_rl b1_channel_probe.py _channels:="1,4" _levels:="0.05" _duration:=3.0
  multi-channel with per-channel levels:
    rosrun albc_rl b1_channel_probe.py _channels:="1,4" _levels:="0.05,0.03" _duration:=3.0
  opposite-sign pair drive (e.g. m0 vs m3 for pitch-torque vertical-sign check):
    rosrun albc_rl b1_channel_probe.py _pair:="0,3" _level:=0.05 _duration:=3.0

  NOTE (2026-07-05 session 2): single-channel driving alone showed only yaw,
  which made m2 look vertical by illusion -- driving channel pairs together
  (multi-channel) or in opposition (pair) is needed to isolate surge/sway from
  yaw and to read pitch-torque vertical sign correctly.
"""
import rospy
from hero_msgs.msg import hero_agent_thruster_cmd
from albc_rl.contract import TOPICS

NUM_THR = 6
RATE_HZ = 20.0   # >3.3 Hz so the firmware 300ms watchdog stays armed


def clamp(level):
    # Clamp to the policy contract [-1, 1] only. The earlier bring-up ceiling
    # (0.3, then 0.5) was removed at the operator's request on 2026-07-05:
    # vertical m0/m3 need more than 0.3 to clear DEPTH_BIAS=30 plus the narrow
    # RL_PWM_SPAN_VERT=150, and the operator drives and judges risk directly.
    # The firmware still constrains PWM to ESC_MIN/MAX and DEPTH_PWM_MIN/MAX,
    # so hardware limits stay enforced downstream regardless of this value.
    # WARNING: near +-1 on a FREE (untethered) robot this is FULL thrust and
    # it moves fast / dives; the operator stops it if it gets risky.
    return max(-1.0, min(1.0, float(level)))


def build_msg(channel, level):
    m = hero_agent_thruster_cmd()
    for i in range(NUM_THR):
        m.thrust[i] = 0.0
    if 0 <= channel < NUM_THR:
        m.thrust[channel] = clamp(level)
    return m


def build_multi_msg(channels, levels):
    """Drive multiple channels simultaneously, each clamped independently.
    channels/levels are already-parsed lists of int/float, same length
    (levels padded by the caller). Out-of-range channel indices are skipped."""
    m = hero_agent_thruster_cmd()
    for i in range(NUM_THR):
        m.thrust[i] = 0.0
    applied = []
    for ch, lvl in zip(channels, levels):
        if 0 <= ch < NUM_THR:
            m.thrust[ch] = clamp(lvl)
            applied.append((ch, m.thrust[ch]))
        else:
            rospy.logwarn("b1_channel_probe: ignoring out-of-range channel %d", ch)
    return m, applied


def build_pair_msg(ch_a, ch_b, level):
    """Drive two channels with opposite sign: ch_a=+level, ch_b=-level."""
    m = hero_agent_thruster_cmd()
    for i in range(NUM_THR):
        m.thrust[i] = 0.0
    lvl = clamp(level)
    applied = []
    if 0 <= ch_a < NUM_THR:
        m.thrust[ch_a] = lvl
        applied.append((ch_a, m.thrust[ch_a]))
    else:
        rospy.logwarn("b1_channel_probe: ignoring out-of-range channel %d", ch_a)
    if 0 <= ch_b < NUM_THR:
        m.thrust[ch_b] = -lvl
        applied.append((ch_b, m.thrust[ch_b]))
    else:
        rospy.logwarn("b1_channel_probe: ignoring out-of-range channel %d", ch_b)
    return m, applied


def _parse_int_list(s):
    """Parse a comma-separated string of ints. Returns [] on empty/blank input.
    Raises ValueError on malformed tokens (caller handles/logs)."""
    s = (s or "").strip()
    if not s:
        return []
    return [int(tok.strip()) for tok in s.split(",")]


def _parse_float_list(s):
    """Parse a comma-separated string of floats. Returns [] on empty/blank input."""
    s = (s or "").strip()
    if not s:
        return []
    return [float(tok.strip()) for tok in s.split(",")]


def main():
    rospy.init_node("b1_channel_probe")
    # Capture whether ~channel was EXPLICITLY passed this run, BEFORE the cleanup
    # loop deletes it. This distinguishes an operator-typed `_channel:=-1` (real
    # panic) from the mere ABSENCE of ~channel (get_param default -1). Without
    # this, running `_pair:=...` with no ~channel would hit the default -1 and the
    # panic-first check below would fire, making pair/channels UNREACHABLE
    # (observed 2026-07-05). Panic must mean "operator asked to stop", not "no
    # channel arg given".
    channel_given = rospy.has_param("~channel")
    channel = int(rospy.get_param("~channel", -1))       # -1 = all neutral
    level = float(rospy.get_param("~level", 0.05))
    duration = float(rospy.get_param("~duration", 3.0))
    # Coerce to str: rosparam may hand back a non-string when the operator
    # forgets the quotes (e.g. `_pair:=0,3` arrives as int/float, `_pair:=5` as
    # int). Without this, `.strip()` below would raise AttributeError -- which is
    # NOT caught by the `except ValueError` guards, so the node would die BEFORE
    # sending NEUTRAL, violating the always-fail-to-NEUTRAL contract. str(... or
    # "") maps a None/empty param to "" and any coerced value to its string form,
    # which then flows through the normal _parse_*_list -> NEUTRAL-on-failure path.
    pair_str = str(rospy.get_param("~pair", "") or "")
    channels_str = str(rospy.get_param("~channels", "") or "")
    levels_str = str(rospy.get_param("~levels", "") or "")

    # CRITICAL: `rosrun ... _p:=v` writes private params to the PARAM SERVER and
    # they PERSIST after the node exits. A later run with a DIFFERENT flag (e.g.
    # `_channel:=-1` panic, or `_channels:=...`) still sees the stale `~pair` from
    # an earlier run and, because mode priority is pair > channels > single, keeps
    # driving the OLD pair -- the panic stop and every new mode are silently
    # overridden (observed 2026-07-05: three different commands all ran the first
    # `_pair:=0,3`). So immediately AFTER reading this run's values, delete our own
    # private params from the server. This run already has its values in local
    # vars, so the delete does not affect it; it only stops the NEXT run from
    # inheriting them. Each rosrun re-writes the params it explicitly passes just
    # before the node starts, so a param given this run survives to be read above.
    for _p in ("~pair", "~channels", "~levels", "~channel", "~level", "~duration"):
        try:
            if rospy.has_param(_p):
                rospy.delete_param(_p)
        except Exception as _e:
            # never let param cleanup abort the run / block the drive, but DO log:
            # a silently-failing cleanup is exactly how the persistence bug stayed
            # invisible for three runs (2026-07-05). The next run may then inherit
            # this param -- the panic-first check below is the structural backstop.
            rospy.logwarn("b1_channel_probe: param cleanup failed for %s: %s "
                          "(next run may inherit it)", _p, _e)

    pub = rospy.Publisher(TOPICS["thruster_pwm"],
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

    # PANIC-FIRST (structural backstop, above all mode priority): an EXPLICIT
    # `_channel:=-1` ALWAYS means STOP and must never be overridable by a leftover
    # ~pair/~channels from a prior run. The per-run param delete above already
    # clears stale state, but if that delete ever fails (logged), this check still
    # guarantees the panic reaches NEUTRAL. Checked BEFORE pair/channels on purpose.
    # Guarded by channel_given so that OMITTING ~channel (default -1) does NOT
    # trigger panic -- otherwise pair/channels modes would be unreachable.
    if channel_given and channel == -1:
        rospy.loginfo("b1_channel_probe: PANIC STOP (channel=-1) -- sending zeros")
        send_neutral()
        return

    # Mode priority: ~pair > ~channels > legacy ~channel single. Any parse
    # failure falls back to ALL-NEUTRAL (panic stop) rather than driving
    # something unintended.
    msg = None
    applied = None

    if pair_str.strip():
        try:
            pair = _parse_int_list(pair_str)
        except ValueError:
            rospy.logerr("b1_channel_probe: malformed ~pair=%r -- ALL-NEUTRAL", pair_str)
            send_neutral()
            return
        if len(pair) != 2:
            rospy.logerr("b1_channel_probe: ~pair must have exactly 2 channels, got %r -- ALL-NEUTRAL",
                         pair_str)
            send_neutral()
            return
        msg, applied = build_pair_msg(pair[0], pair[1], level)
        if not applied:
            rospy.logerr("b1_channel_probe: ~pair=%r had no valid channels -- ALL-NEUTRAL", pair_str)
            send_neutral()
            return
        rospy.logwarn("b1_channel_probe: PAIR drive m%d=+%.3f / m%d=%.3f for %.1fs "
                      "(watch for PITCH torque / vertical sign)",
                      pair[0], clamp(level), pair[1], -clamp(level), duration)

    elif channels_str.strip():
        try:
            channels = _parse_int_list(channels_str)
            levels = _parse_float_list(levels_str) if levels_str.strip() else [level]
        except ValueError:
            rospy.logerr("b1_channel_probe: malformed ~channels=%r / ~levels=%r -- ALL-NEUTRAL",
                         channels_str, levels_str)
            send_neutral()
            return
        if not channels:
            rospy.logerr("b1_channel_probe: ~channels=%r parsed empty -- ALL-NEUTRAL", channels_str)
            send_neutral()
            return
        # pad levels: repeat last value (or single value for all channels)
        if len(levels) < len(channels):
            levels = levels + [levels[-1]] * (len(channels) - len(levels))
        msg, applied = build_multi_msg(channels, levels)
        if not applied:
            rospy.logerr("b1_channel_probe: ~channels=%r had no valid channels -- ALL-NEUTRAL",
                         channels_str)
            send_neutral()
            return
        applied_str = ", ".join("m%d=%.3f" % (ch, lvl) for ch, lvl in applied)
        rospy.logwarn("b1_channel_probe: MULTI-CHANNEL drive %s for %.1fs "
                      "(watch which axis moves: surge/sway vs yaw)",
                      applied_str, duration)

    elif 0 <= channel < NUM_THR:
        msg = build_msg(channel, level)
        rospy.logwarn("b1_channel_probe: driving m%d at %.3f for %.1fs "
                      "(watch physical motor m%d: is it VERTICAL/HORIZONTAL? which way?)",
                      channel, msg.thrust[channel], duration, channel)

    else:
        rospy.loginfo("b1_channel_probe: ALL-NEUTRAL (channel=%d) -- sending zeros", channel)
        send_neutral()
        return

    rate = rospy.Rate(RATE_HZ)
    t_end = rospy.Time.now() + rospy.Duration.from_sec(duration)
    while not rospy.is_shutdown() and rospy.Time.now() < t_end:
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("b1_channel_probe: duration elapsed -> NEUTRAL")
    send_neutral()


if __name__ == "__main__":
    main()
