# ADR 001 — Joint command baseline: the 2026-08-13 cable break, and why the clamp is gone

Status: accepted
Code: `robot/albc_control/include/albc_control/joint_unwrap.h`,
`robot/albc_rl/src/albc_rl/np_policy.py` (target accumulation),
`robot/albc_control/src/joint_angle_command.cpp` (`~joint1_abort_rad`)
Test: `tests/characterization/test_joint_unwrap.cpp`

Two decisions from the same incident, moved out of the sources verbatim on
2026-09-04. The code keeps the conventions; this keeps the evidence.

## 1. unwrapNearest, and the bug it replaced

joint_unwrap.h — pure oracle for the joint command baseline arithmetic.

Extracted from joint_angle_command.cpp so tests/characterization/run.sh can
exercise it with a bare compiler (no ROS, no catkin), the same way
imu_rotation.h and albc_kinematics.h are exercised.

WHY THIS EXISTS — the 2026-08-13 J1->J2 cable break
---------------------------------------------------------------------------
The driver follows the command topic by accumulating unwrapped deltas into a
running absolute_angle. Its init seeded the two halves in DIFFERENT
representations of the same angle:
    absolute_angle = angle0                         (cumulative encoder value)
    prev_commanded = fmod(angle0, 2pi) in [0, 2pi)  (WRAPPED)
They differ by k*2pi. The RL node's first command is the CUMULATIVE measured
angle, so the first delta is k*2pi -- and the old unwrap removed at most ONE
2pi, leaking (k-1) turns straight into absolute_angle on that first command.

Recomputed from the 14 recorded bags of that session this fired at +1, -1 and
-3 turns. On the last run the driver's goal was -37.6 rad while the policy had
asked for -18.755 -- a value the policy CANNOT produce (its own accumulator is
bounded). J1 stalled at -35.54 rad with HW error 0x20 OVERLOAD, having twisted
the J1->J2 daisy-chain cable apart.

It hid for months because in the normal case the bug returns EXACTLY ZERO: for
|angle0| < 2pi the two representations differ by exactly one 2pi and the
single-step unwrap cancels it perfectly. 11 of those 14 runs show 0.000. An
error that is conditionally zero cannot be found by coverage -- only by
feeding it an out-of-band input, which is what test_joint_unwrap.cpp does.

unwrapNearest() removes ANY number of turns, so the first-command residual is
0 for every k, and it is IDENTICAL to the old rule whenever |delta| < pi --
which is every steady-state tick (the RL accumulator moves at most
DELTA_SCALE = 0.10 rad/tick, and the classic publisher is continuous).

## 2. Why the 6*pi clamp on the joint target was removed

Reference band on joint1's accumulated target, for documentation and tests.
4*pi is the TRAINING constraint (joint1_position_cost, limit_rad = 4*pi,
budget 0.01 in the deployed run's params/env.yaml). It is NOT enforced here.

2026-08-17: the former np.clip at 6*pi is REMOVED, deliberately.
  1. It hid the metric. Truncating the target destroys "how far did this
     controller actually try to go", which is exactly the quantity used to
     compare TDC / classic PID / RL. The driver now COUNTS it instead, on
     /albc/joint_guard, where all controllers pass through one instrument.
  2. It manufactured a step command. On a seed already outside the band the
     clip turned the first published target into a multi-radian jump
     (measured 2026-08-13: seed -21.953 -> first command -18.755, a 3.2 rad
     step), and no unwrap rule can absorb a jump larger than pi.
  3. The sim has no clamp either (albc_env.py _joint_pos_targets), so this was
     a deploy-only divergence from training.
The physical ceiling now lives one layer down, in joint_angle_command
(~joint1_abort_rad, default 6*pi = 3 turns), which ABORTS the run rather than
silently truncating it -- and checks the measured angle, not just the command.
