// Golden oracle: albc_controller feedback pipeline math (ROS-free extraction)
//
// 1:1 transcription of albc_controller.cpp:674-702 — the asymmetric-damping
// gate + integral anti-windup (freeze + clamp) + derivative 1st-order LPF that
// run inside the main feedback loop for the TDC / PID / FIXED modes.
//
// The ROS plumbing (state struct, loop_rate, dynamic_reconfigure) is dropped.
// Every value the math reads is passed in explicitly. Operators, signs,
// constants, and evaluation order are preserved CHARACTER-FOR-CHARACTER from
// the source so the redesign can be checked for byte-identical behavior.
//
// Sign / freeze conventions pinned here (do NOT "fix" without intent):
//   - integral ACCUMULATES only while |angle| >= LEVEL_THRESHOLD; below the
//     threshold the robot is "level" and the integral is FROZEN (held, not
//     reset) to prevent windup at equilibrium.
//   - clamp is symmetric: std::max(-INTEGRAL_MAX, std::min(INTEGRAL_MAX, x)).
//   - asymmetric damping gate is applied to the RAW derivative (before LPF):
//     error * raw_deriv < 0  =>  raw_deriv := 0  (system converging, kill D).
//     error * raw_deriv > 0  =>  pass through    (growing / overshoot, keep D).
//   - LPF: out = alpha*raw + (1-alpha)*prev_filtered.

#ifndef DAMPING_INTEGRAL_ORACLE_H
#define DAMPING_INTEGRAL_ORACLE_H

#include <algorithm>   // std::max, std::min
#include <cmath>       // std::abs

// Current production constants (albc_controller.cpp:71,77,78).
// Exposed so tests don't re-type magic numbers; the functions take them as
// parameters to stay decoupled from any future tuning.
static const double ORACLE_INTEGRAL_MAX    = 100.0;
static const double ORACLE_DERIV_LPF_ALPHA = 0.2;
static const double ORACLE_LEVEL_THRESHOLD = 0.01745; // rad (1 deg)

// Integral accumulation with freeze-when-level + symmetric anti-windup clamp.
// Pure transcription of albc_controller.cpp:674-677 (roll) / 678-681 (pitch);
// roll and pitch are byte-identical so one function serves both.
//
//   integral      : current accumulated integral (state.integral_*)
//   error         : state.error_* (= target - current)
//   current_angle : state.current_* (roll or pitch), used for the level gate
// Returns the NEW integral value.
inline double integralStep(double integral, double error, double current_angle,
                           double LEVEL_THRESHOLD, double INTEGRAL_MAX)
{
    if (std::abs(current_angle) >= LEVEL_THRESHOLD) {
        integral += error;
        integral  = std::max(-INTEGRAL_MAX, std::min(INTEGRAL_MAX, integral));
    }
    return integral;
}

// Asymmetric-damping gate + derivative low-pass filter.
// Pure transcription of albc_controller.cpp:684,685 (raw deriv) + 692-694 /
// 695-697 (gate) + 699,700 (LPF). roll and pitch are byte-identical.
//
//   error                   : state.error_*
//   prev_error              : state.prev_error_*
//   raw_deriv_prev_filtered : state.filtered_deriv_* (previous tick's LPF output)
//   dt                      : 1.0 / loop_rate_hz
//   alpha                   : DERIV_LPF_ALPHA
//   out_filtered            : receives the new filtered derivative (= state.filtered_deriv_*)
// Returns the derivative term fed to the control law (== out_filtered).
inline double dampedDerivative(double error, double prev_error,
                               double raw_deriv_prev_filtered, double dt,
                               double alpha, double& out_filtered)
{
    double raw_deriv = (error - prev_error) / dt;

    // Gate on RAW (before LPF): error * d_error/dt < 0 => converging => kill D.
    if (error * raw_deriv < 0) {
        raw_deriv = 0.0;
    }

    double derivative = alpha * raw_deriv + (1.0 - alpha) * raw_deriv_prev_filtered;
    out_filtered = derivative;
    return derivative;
}

#endif // DAMPING_INTEGRAL_ORACLE_H
