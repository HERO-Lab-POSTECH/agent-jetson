// Production header: albc_controller computeControlOutput (ROS-free)
//
// Byte-identical to albc_controller.cpp:385-433 (the Control Law switch).
// The ROS plumbing is dropped; the global `state` / `gains` structs and the
// `dt`/`derivative_*` parameters are flattened into an explicit CtrlIn struct.
// Every variable name, sign, constant, and operation order is preserved
// byte-for-byte from the source.
//
// Constants below are copied verbatim from albc_controller.cpp:72-78:
//   COS_EPSILON       = 1e-6        (line 72)
//   COMMON_FACTOR_MAX = 10.0        (line 73)
//   PID_BASE_X        = -L2         (line 75)
//   PID_BASE_Y        =  L1         (line 76)
//   LEVEL_THRESHOLD   = 0.01745     (line 78)
//   FIXED_ALPHA       = 0.05        (line 424, function-local constexpr)
// Fb / L1 / L2 come from the pure albc_kinematics.h (no ROS).
//
// mode enum (matches ControlMode): 1=TDC, 2=PID, 3=FIXED, 4=MANUAL.
// Do NOT "fix" signs (e.g. dx = -common_factor*..., the (-1.0) on pitch PID,
// the (-L2 - target_x) on FIXED) without intent — they are the pinned behavior.
//
// DELIBERATE DIVERGENCE FROM THE ORIGINAL (2026-08-12) — the ONE thing in this
// file that is no longer byte-identical to albc_controller.cpp:385-433:
//
//   the LEVEL_THRESHOLD gates now test the ERROR, not the measured angle.
//     TDC: |error_roll| / |error_pitch|   (was |current_roll| / |current_pitch|)
//     PID: same swap on both branches
//   feedback_filters.h integralStep() changed identically, same date.
//
// WHY. error = target - current, so at target 0 the two forms are algebraically
// IDENTICAL (|error| == |current|) -- every run this robot has ever done was at
// target 0, which is why the original never misbehaved. The moment a nonzero
// target_roll/target_pitch is set, gating on the measured angle makes those two
// parameters DEAD: a level robot (|current| < 1 deg) clamps dy/dx to zero before
// the target is ever consulted, so it can never be commanded away from level,
// and once tilted the integrator has no setpoint to stop at. Measured live in
// the tank 2026-08-12: target_roll = 15 deg with the robot at roll 0.52 deg
// produced exactly zero arm motion.
//
// This change is therefore a no-op for all historical target-0 behavior and only
// adds meaning to a nonzero setpoint. Revert both files together if it is ever
// reverted -- the integral gate and the control gate must agree.

#ifndef ALBC_CONTROL_CONTROL_LAW_H
#define ALBC_CONTROL_CONTROL_LAW_H

#include <cmath>
#include <algorithm>   // std::min, std::max
#include "albc_control/albc_kinematics.h"

using albc::Fb;
using albc::L1;
using albc::L2;

// Constants — verbatim from albc_controller.cpp:72-78 (and 424 for FIXED_ALPHA).
static constexpr double COS_EPSILON          = 1e-6;
static constexpr double COMMON_FACTOR_MAX    = 10.0;
static constexpr double PID_BASE_X           = -L2;
static constexpr double PID_BASE_Y           =  L1;
static constexpr double LEVEL_THRESHOLD      = 0.01745;

// Mode codes (mirror ControlMode in albc_controller.cpp).
enum { CTRL_TDC = 1, CTRL_PID = 2, CTRL_FIXED = 3, CTRL_MANUAL = 4 };

// Flattened inputs: global `state` + `gains` + the derivative_* params.
struct CtrlIn {
    int    mode;
    double current_roll, current_pitch;
    double error_roll,   error_pitch;
    double integral_roll, integral_pitch;
    double deriv_roll,   deriv_pitch;     // = derivative_roll / derivative_pitch params
    double target_x, target_y;            // in/out: prior setpoint
    double M_td, Kp_td;                   // gains.M_td, gains.Kp_td
    double kp_roll, ki_roll, kd_roll;     // gains.* (roll PID)
    double kp_pitch, ki_pitch, kd_pitch;  // gains.* (pitch PID)
};

struct CtrlOut {
    double target_x, target_y;
};

// Pure transcription of computeControlOutput (albc_controller.cpp:385-433).
inline CtrlOut computeControlOutputOracle(const CtrlIn& in) {
    // Working copy of the mutated state fields.
    double target_x = in.target_x;
    double target_y = in.target_y;

    switch (in.mode) {
    case CTRL_TDC: {
        double denominator = std::abs(Fb * cos(in.current_roll) * cos(in.current_pitch));
        double common_factor = std::min(
            Fb / std::max(denominator, COS_EPSILON),
            COMMON_FACTOR_MAX);

        double dy =  common_factor * (in.M_td * in.Kp_td * in.error_roll);
        double dx = -common_factor * (in.M_td * in.Kp_td * in.error_pitch);

        // Equilibrium hold: within LEVEL_THRESHOLD of the TARGET, clamp dy/dx to 0.
        // Gated on the ERROR, not on the measured angle -- see the 2026-08-12 note
        // in the header block. At target 0 the two are identical.
        if (std::abs(in.error_roll)  < LEVEL_THRESHOLD) dy = 0.0;
        if (std::abs(in.error_pitch) < LEVEL_THRESHOLD) dx = 0.0;

        target_y += dy;
        target_x += dx;
        break;
    }
    case CTRL_PID:
        if (std::abs(in.error_roll) >= LEVEL_THRESHOLD) {
            target_y = PID_BASE_Y + in.kp_roll  * in.error_roll
                             + in.ki_roll  * in.integral_roll
                             + in.kd_roll  * in.deriv_roll;
        }
        if (std::abs(in.error_pitch) >= LEVEL_THRESHOLD) {
            target_x = PID_BASE_X + (-1.0) * (in.kp_pitch * in.error_pitch
                             + in.ki_pitch * in.integral_pitch
                             + in.kd_pitch * in.deriv_pitch);
        }
        break;

    case CTRL_FIXED: {
        constexpr double FIXED_ALPHA = 0.05;
        target_x += FIXED_ALPHA * (-L2 - target_x);
        target_y += FIXED_ALPHA * ( L1 - target_y);
        break;
    }

    case CTRL_MANUAL:
        break;
    }

    CtrlOut out;
    out.target_x = target_x;
    out.target_y = target_y;
    return out;
}

#endif // ALBC_CONTROL_CONTROL_LAW_H
