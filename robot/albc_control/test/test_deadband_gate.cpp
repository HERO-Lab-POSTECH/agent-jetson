// Deadband-gate guard: the LEVEL_THRESHOLD gates must key off the ERROR.
//
// WHY THIS EXISTS
// ---------------
// Until 2026-08-12 the TDC clamp, the PID update gate, and the integral gate all
// tested the MEASURED angle (|current_roll|) instead of the error. Because
// error = target - current, the two forms are algebraically identical whenever
// the target is 0 -- and every run this robot had ever done was at target 0. So
// the defect was invisible until a nonzero target_roll was set in the tank
// (2026-08-12: target_roll = 15 deg, robot at roll 0.52 deg, arm did not move at
// all, because |current| < 1 deg clamped dy to zero before the target was ever
// consulted). target_roll / target_pitch were dead parameters.
//
// This file pins BOTH halves of that fix:
//   1. REGRESSION: at target 0 the new gate is bit-for-bit the old gate, so no
//      historical behavior changed. Checked over a swept range, not one point.
//   2. THE FIX: at a nonzero target with a level robot, the old gate produced
//      exactly zero and the new gate produces motion.
//
// No framework, no ROS, no catkin wiring -- build and run it directly:
//   g++ -std=c++11 -I../include test_deadband_gate.cpp -o /tmp/tdg && /tmp/tdg
// (from robot/albc_control/test/)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

#include "albc_control/control_law.h"
#include "albc_control/feedback_filters.h"

static int failures = 0;

static void check(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL %s\n", what); ++failures; }
    else     { std::printf("PASS %s\n", what); }
}

// The gates exactly as they were BEFORE 2026-08-12, for the regression compare.
static double oldTdcDy(double current_roll, double error_roll,
                       double current_pitch, double M_td, double Kp_td) {
    double denominator = std::abs(Fb * cos(current_roll) * cos(current_pitch));
    double common_factor = std::min(Fb / std::max(denominator, COS_EPSILON),
                                    COMMON_FACTOR_MAX);
    double dy = common_factor * (M_td * Kp_td * error_roll);
    if (std::abs(current_roll) < LEVEL_THRESHOLD) dy = 0.0;   // OLD: gates on current
    return dy;
}

static double oldIntegral(double integral, double error, double current_angle) {
    if (std::abs(current_angle) >= ORACLE_LEVEL_THRESHOLD) {  // OLD: gates on current
        integral += error;
        integral = std::max(-ORACLE_INTEGRAL_MAX,
                            std::min(ORACLE_INTEGRAL_MAX, integral));
    }
    return integral;
}

// One TDC step through the production path; returns the applied dy (= dTarget_y).
static double newTdcDy(double target_roll, double current_roll,
                       double current_pitch, double M_td, double Kp_td) {
    CtrlIn in = CtrlIn();
    in.mode          = CTRL_TDC;
    in.current_roll  = current_roll;
    in.current_pitch = current_pitch;
    in.error_roll    = target_roll - current_roll;
    in.error_pitch   = 0.0 - current_pitch;
    in.target_x      = 0.0;
    in.target_y      = 0.0;
    in.M_td          = M_td;
    in.Kp_td         = Kp_td;
    CtrlOut out = computeControlOutputOracle(in);
    return out.target_y;   // target_y started at 0, so this IS dy
}

int main() {
    const double M_td = 0.004, Kp_td = 0.04;   // deployed values (albc_controller.yaml)
    const double DEG = M_PI / 180.0;

    // ---- 1. REGRESSION: at target 0, new == old across the whole sweep -------
    // Swept rather than spot-checked: the gate boundary sits at exactly 1 deg, so
    // a coarse sample could straddle it and still agree by luck.
    bool all_equal = true;
    double worst = 0.0;
    for (double roll_deg = -45.0; roll_deg <= 45.0; roll_deg += 0.05) {
        double cr = roll_deg * DEG;
        double newv = newTdcDy(0.0, cr, 0.0, M_td, Kp_td);
        double oldv = oldTdcDy(cr, 0.0 - cr, 0.0, M_td, Kp_td);
        double d = std::abs(newv - oldv);
        if (d > worst) worst = d;
        if (d != 0.0) all_equal = false;
    }
    std::printf("     (target 0 sweep -45..45 deg, max |new-old| = %.3e)\n", worst);
    check(all_equal, "target 0: TDC gate is bit-identical to the pre-fix gate");

    bool integ_equal = true;
    for (double roll_deg = -45.0; roll_deg <= 45.0; roll_deg += 0.05) {
        double cr = roll_deg * DEG;
        double err = 0.0 - cr;                       // target 0
        if (integralStep(7.0, err, ORACLE_LEVEL_THRESHOLD, ORACLE_INTEGRAL_MAX)
            != oldIntegral(7.0, err, cr)) integ_equal = false;
    }
    check(integ_equal, "target 0: integral gate is bit-identical to the pre-fix gate");

    // ---- 2. THE FIX: nonzero target on a level robot now commands motion ------
    // The exact tank condition that exposed the defect.
    const double tank_roll = 0.523 * DEG;            // measured live 2026-08-12
    const double tgt15     = 15.0 * DEG;
    double old_dy = oldTdcDy(tank_roll, tgt15 - tank_roll, -0.669 * DEG, M_td, Kp_td);
    double new_dy = newTdcDy(tgt15, tank_roll, -0.669 * DEG, M_td, Kp_td);
    std::printf("     (target 15 deg @ roll 0.523 deg: old dy = %.3e, new dy = %.3e)\n",
                old_dy, new_dy);
    check(old_dy == 0.0,  "pre-fix gate produced ZERO motion for a 15 deg target (the bug)");
    check(new_dy > 0.0,   "post-fix gate produces positive dy toward a +15 deg target");

    // Sign must follow the target: a negative target drives the arm the other way.
    check(newTdcDy(-tgt15, tank_roll, -0.669 * DEG, M_td, Kp_td) < 0.0,
          "post-fix gate reverses dy for a -15 deg target");

    // ---- 3. It still holds AT the setpoint (no chatter at the new equilibrium) -
    check(newTdcDy(tgt15, tgt15, 0.0, M_td, Kp_td) == 0.0,
          "at the setpoint the clamp fires: dy == 0");
    check(newTdcDy(tgt15, 14.5 * DEG, 0.0, M_td, Kp_td) == 0.0,
          "within 1 deg of the setpoint the clamp still fires: dy == 0");
    check(newTdcDy(tgt15, 13.5 * DEG, 0.0, M_td, Kp_td) > 0.0,
          "beyond 1 deg from the setpoint the clamp releases");

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASS",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
