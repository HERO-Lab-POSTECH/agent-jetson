// joint_unwrap.h — pure oracle for the joint command baseline arithmetic.
//
// Extracted from joint_angle_command.cpp so tests/characterization/run.sh can
// exercise it with a bare compiler (no ROS, no catkin), the same way
// imu_rotation.h and albc_kinematics.h are exercised.
//
// WHY THIS EXISTS — the 2026-08-13 J1->J2 cable break
// ---------------------------------------------------------------------------
// The driver follows the command topic by accumulating unwrapped deltas into a
// running absolute_angle. Its init seeded the two halves in DIFFERENT
// representations of the same angle:
//     absolute_angle = angle0                         (cumulative encoder value)
//     prev_commanded = fmod(angle0, 2pi) in [0, 2pi)  (WRAPPED)
// They differ by k*2pi. The RL node's first command is the CUMULATIVE measured
// angle, so the first delta is k*2pi -- and the old unwrap removed at most ONE
// 2pi, leaking (k-1) turns straight into absolute_angle on that first command.
//
// Recomputed from the 14 recorded bags of that session this fired at +1, -1 and
// -3 turns. On the last run the driver's goal was -37.6 rad while the policy had
// asked for -18.755 -- a value the policy CANNOT produce (its own accumulator is
// bounded). J1 stalled at -35.54 rad with HW error 0x20 OVERLOAD, having twisted
// the J1->J2 daisy-chain cable apart.
//
// It hid for months because in the normal case the bug returns EXACTLY ZERO: for
// |angle0| < 2pi the two representations differ by exactly one 2pi and the
// single-step unwrap cancels it perfectly. 11 of those 14 runs show 0.000. An
// error that is conditionally zero cannot be found by coverage -- only by
// feeding it an out-of-band input, which is what test_joint_unwrap.cpp does.
//
// unwrapNearest() removes ANY number of turns, so the first-command residual is
// 0 for every k, and it is IDENTICAL to the old rule whenever |delta| < pi --
// which is every steady-state tick (the RL accumulator moves at most
// DELTA_SCALE = 0.10 rad/tick, and the classic publisher is continuous).
#ifndef ALBC_CONTROL_JOINT_UNWRAP_H
#define ALBC_CONTROL_JOINT_UNWRAP_H

#include <cmath>

namespace albc {

// Fold a command-vs-baseline difference into (-pi, pi], removing however many
// whole turns separate the two. The ceil(x - 0.5) form (rather than
// floor(x + 0.5)) reproduces the old rule's boundary convention exactly:
// delta == +pi stays +pi, delta == -pi maps to +pi.
inline double unwrapNearest(double delta)
{
    return delta - 2.0 * M_PI * std::ceil(delta / (2.0 * M_PI) - 0.5);
}

// The OLD single-step rule. NOT used by the driver any more -- kept so the
// regression test can reproduce the historic arithmetic on the recorded data
// and show what it did, instead of asserting it from memory.
inline double unwrapSingleStep(double delta)
{
    if (delta > M_PI) return delta - 2.0 * M_PI;
    if (delta <= -M_PI) return delta + 2.0 * M_PI;
    return delta;
}

// The driver's init used to wrap the command baseline into [0, 2pi). Kept for
// the same reason as unwrapSingleStep: the test replays the historic init.
inline double wrapTo2Pi(double x)
{
    double w = std::fmod(x, 2.0 * M_PI);
    if (w < 0.0) w += 2.0 * M_PI;
    return w;
}

// |angle| > limit, where a non-positive limit means "guard disabled".
// Used for the driver-layer cable ceiling, which ABORTS rather than clamping:
// clamping would hide how far the controller actually wanted to go, and that
// quantity is the metric used to compare TDC / classic PID / RL.
inline bool overGuard(double angle, double limit)
{
    return limit > 0.0 && std::fabs(angle) > limit;
}

}  // namespace albc

#endif  // ALBC_CONTROL_JOINT_UNWRAP_H
