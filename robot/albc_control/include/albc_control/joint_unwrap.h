// joint_unwrap.h — pure oracle for the joint command baseline arithmetic.
//
// Extracted from joint_angle_command.cpp so tests/characterization/run.sh can
// exercise it with a bare compiler (no ROS, no catkin), the same way
// imu_rotation.h and albc_kinematics.h are exercised.
//
// CONVENTION (do NOT "fix" without intent): unwrapNearest() removes ANY number
// of whole turns, not at most one. That is what makes the first command after a
// seed exact for every k, and it is IDENTICAL to the old single-step rule
// whenever |delta| < pi -- which is every steady-state tick.
//
// It replaced a rule that leaked turns into the driver's baseline and twisted
// the J1->J2 cable apart on 2026-08-13. The failure returns exactly zero in the
// normal case, which is why it hid for months and why the out-of-band cases in
// test_joint_unwrap.cpp are the only thing that catches it:
//   docs/adr/001-joint-unwrap-cable-break.md
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
