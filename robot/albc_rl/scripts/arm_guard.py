#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Pure arm-protection predicates. NO rospy -- that is the whole point of the file.

WHY IT EXISTS. These two functions are the arithmetic behind the guards that were
missing when arm2 fractured on 2026-08-25, and they are the most intricate logic in
that change. While they lived inside rl_inference_node.py their tests could only run
on the board, because importing them dragged in rospy -- so the regression test that
stands in for the CRITICAL current-read defect ran nowhere else. Neither function
needs ROS. Splitting them out is what lets test_arm_guard.py check them anywhere,
which is where drift actually gets caught.

Each is also shared by two call sites (the start-state gate and the per-tick guard),
so keeping them in one place is what makes those call sites structurally incapable
of disagreeing about what "unsafe" means.

Incident and calibration: .community/posts/finding/047-e2-run1-arm2-fracture.md
"""
import numpy as np


def j2_in_window(theta2_rad, lo_rad, hi_rad):
    """Is joint2 inside the safe window? theta2 is WRAPPED to [0, 2*pi) first.

    Wrapping is mandatory, not cosmetic. /albc/joint_states and the policy's own
    joint-target accumulator use different representations of the same physical
    pose -- the driver logged "command -0.061 rad is -1.00 turns from baseline
    6.107" on the run that broke arm2, i.e. -0.061 and 6.222 are the same place.
    Comparing an unwrapped accumulator against a fixed window would reject poses
    physically identical to accepted ones, and would drift further out every turn.

    The window is ONE interval: the [0, pi] elbow branch, minus asin(w^2) at each
    end. The stated premise |sin theta2| >= w^2 is satisfied on a second interval
    too ([185.16, 354.84] deg, where |sin| is maximal at 270), and that mirror
    branch is deliberately EXCLUDED -- it is the other elbow solution and out of
    the trained distribution. Do not widen the window to it on the strength of the
    manipulability argument alone.

    hi <= lo disables the window (returns True), matching the driver's
    overGuard(limit > 0) convention for "this check is off".
    """
    if hi_rad <= lo_rad:
        return True
    th = float(np.mod(float(theta2_rad), 2.0 * np.pi))
    return lo_rad <= th <= hi_rad


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
    assert j2_in_window(2.9, 0.09016, 3.05143)
    assert not j2_in_window(-0.061, 0.09016, 3.05143)
    assert over_current_held(None, 1.0, 1300.0, 900.0, False) == (1.0, 0.0)
    assert over_current_held(1.0, 2.0, 1300.0, 900.0, False)[1] == 1.0
    assert over_current_held(1.0, 2.0, 100.0, 900.0, False) == (None, 0.0)
    assert over_current_held(1.0, 2.0, 100.0, 900.0, True)[1] == 1.0
    assert over_current_held(100.0, 50.0, 1300.0, 900.0, False) == (50.0, 0.0)
    print("arm_guard smoke OK")
