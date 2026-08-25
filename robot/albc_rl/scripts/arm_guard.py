#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Pure arm-protection predicate. NO rospy -- that is the whole point of the file.

WHY IT EXISTS. This is the arithmetic behind the over-current guard that was
missing when arm2 fractured on 2026-08-25, and it is the most intricate logic in
that change. While it lived inside rl_inference_node.py its tests could only run
on the board, because importing it dragged in rospy -- so the regression test that
stands in for the CRITICAL current-read defect ran nowhere else. It does not need
ROS. Splitting it out is what lets test_arm_guard.py check it anywhere, which is
where drift actually gets caught.

Incident and calibration: .community/posts/finding/047-e2-run1-arm2-fracture.md

2026-08-26 (decision/061 A1/A2, guard rollback): this file used to also hold
j2_in_window, the predicate behind a hard theta2 window checked at both the
start-state gate and the per-tick guard. That window is REMOVED, not just moved --
this system is constrained RL (ConstraintTRPO + IPO) and singularity avoidance is
already a TRAINED cost (manipulability_cost, w = sqrt|sin theta2| >= 0.3); sim
never clamps theta2 either. The hand-written clamp this replaced LATCHED the
policy output during an ordinary attitude-lowering move on 2026-08-25 (policy
lifetime 0.255 s, 6 commanded ticks -- notes/2026-08-25-guard-session-retraction-
handoff.md, NOT finding/047, which decision/061 mis-cites for this figure) and
forbade the mirror branch [185.16, 354.84] deg with no argument beyond
"unreviewed." See decision/061.
"""


def over_current_held(prev_since, now, value, cap, stale):
    """Accumulate how long the joint current has been over `cap`.

    Returns (since, held): `since` is the timestamp the excess began (None = clear),
    `held` the seconds it has lasted.

    A SILENT signal is UNKNOWN, not CLEAR. The 2026-08-25 defect was upstream of
    here: the driver published 0 mA on a failed Dynamixel read, so one bad read per
    window reset the accumulator and the guard could never trip while the arm
    stalled -- and a stall is exactly what jams that bus (295/572 read failures
    measured 2026-08-20). The driver now skips the publish instead, and `stale`
    carries that gap through: while it is set, an existing excess keeps counting.
    Only a FRESH under-cap sample clears it.

    The two halves only work as a PAIR. Against a driver that still publishes
    fabricated zeros, `stale` is never true, this returns clear on every zero, and
    the hardening above buys exactly nothing. The driver must be restarted -- out
    of the water -- before any of it is live.

    A BACKWARD clock jump RESTARTS the window rather than freezing it. This board
    restores its clock from a snapshot at boot and can jump days when
    jetson_clock_sync lands; preserving `since` across that would clamp held to 0
    until wall time caught up, i.e. an inert guard for the jump duration. Restarting
    costs one extra cap-length before the trip, which is the fail-safe direction.
    """
    if prev_since is not None and now < prev_since:
        prev_since = None
    if value > cap or (stale and prev_since is not None):
        since = now if prev_since is None else prev_since
        return since, min(max(now - since, 0.0), 1e4)
    return None, 0.0


if __name__ == "__main__":
    # smoke check: python arm_guard.py
    assert over_current_held(None, 1.0, 1300.0, 900.0, False) == (1.0, 0.0)
    assert over_current_held(1.0, 2.0, 1300.0, 900.0, False)[1] == 1.0
    assert over_current_held(1.0, 2.0, 100.0, 900.0, False) == (None, 0.0)
    assert over_current_held(1.0, 2.0, 100.0, 900.0, True)[1] == 1.0
    assert over_current_held(100.0, 50.0, 1300.0, 900.0, False) == (50.0, 0.0)
    print("arm_guard smoke OK")
