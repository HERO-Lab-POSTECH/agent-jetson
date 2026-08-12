// Production header: AttitudeController — feedback pipeline + 4-mode control law.
//
// Absorbs the former global `ControlState` (albc_controller.cpp:113-124),
// `ControlGains` (:87-111), `computeControlOutput()` (:325-373), the main-loop
// feedback block (:598-644) and the mode-change reset block (:533-556).
//
// The control math is NOT re-implemented here — it delegates to the Task-1/2/3
// oracles so behavior stays byte-identical:
//   - feedback_filters.h : integralStep (freeze+clamp), dampedDerivative (gate+LPF)
//   - control_law.h      : computeControlOutputOracle (4-mode switch)
// Only "where state lives" changes (global structs -> class members); never the
// order, signs, constants, or evaluation sequence of the math.
//
// Constants come from the oracle headers (feedback_filters.h ORACLE_* and
// control_law.h LEVEL_THRESHOLD), which are byte-identical to the former
// file-scope constexpr INTEGRAL_MAX / DERIV_LPF_ALPHA / LEVEL_THRESHOLD
// (albc_controller.cpp:73,78,79). DERIV_LPF_ALPHA was also reused by the
// angular-velocity LPF (:566-568), so the same ORACLE_DERIV_LPF_ALPHA is used.
//
// ROS-free: includes only the pure oracle headers (no ros/ros.h, no std_msgs).

#ifndef ALBC_CONTROL_ATTITUDE_CONTROLLER_H
#define ALBC_CONTROL_ATTITUDE_CONTROLLER_H

#include <algorithm>   // std::max, std::min
#include <cmath>       // std::abs
#include "albc_control/control_law.h"       // computeControlOutputOracle, CtrlIn/CtrlOut, LEVEL_THRESHOLD
#include "albc_control/feedback_filters.h"  // integralStep, dampedDerivative, ORACLE_*

namespace albc {

class AttitudeController {
public:
    // ---- Gains (absorbs ControlGains, albc_controller.cpp:87-111) ----------

    // Set the base gains + multiplier, then derive the active gains.
    // Mirrors the main() param load (:427-439,460) and reconfigureCallback
    // (:164-177): base values are stored, then applyMultiplier() runs.
    //
    // The TDC D-term was removed in df46b09 (computeControlOutputOracle no longer
    // uses Kd_td), so the Kd_td gain is gone from this signature — its removal is
    // behavior-neutral. Re-introduction (if ever needed) → see git history.
    void setGains(double M_td_base, double Kp_td_base,
                  double kp_roll_base, double ki_roll_base, double kd_roll_base,
                  double kp_pitch_base, double ki_pitch_base, double kd_pitch_base,
                  double gain_mult) {
        M_td_base_  = M_td_base;  Kp_td_base_ = Kp_td_base;
        kp_roll_base_  = kp_roll_base;   ki_roll_base_  = ki_roll_base;   kd_roll_base_  = kd_roll_base;
        kp_pitch_base_ = kp_pitch_base;  ki_pitch_base_ = ki_pitch_base;  kd_pitch_base_ = kd_pitch_base;
        gain_mult_ = gain_mult;
        applyMultiplier();
    }

    // Recompute active gains from base * gain_mult.
    // Byte-identical to ControlGains::applyMultiplier (albc_controller.cpp:100-110).
    void applyMultiplier() {
        M_td_  = M_td_base_  * gain_mult_;
        Kp_td_ = Kp_td_base_ * gain_mult_;
        kp_roll_  = kp_roll_base_  * gain_mult_;
        ki_roll_  = ki_roll_base_  * gain_mult_;
        kd_roll_  = kd_roll_base_  * gain_mult_;
        kp_pitch_ = kp_pitch_base_ * gain_mult_;
        ki_pitch_ = ki_pitch_base_ * gain_mult_;
        kd_pitch_ = kd_pitch_base_ * gain_mult_;
    }

    // Set roll/pitch target attitude [rad]. Mirrors reconfigureCallback
    // (:161-162) writing state.target_roll / state.target_pitch.
    void setTargets(double target_roll, double target_pitch) {
        target_roll_  = target_roll;
        target_pitch_ = target_pitch;
    }

    // Reset both integrals (anti-windup on gain change).
    // Mirrors reconfigureCallback (:189-190).
    void resetIntegrals() {
        integral_roll_  = 0.0;
        integral_pitch_ = 0.0;
    }

    // ---- Setpoint seed (absorbs main() :473-474 and the per-mode resets) ----

    // Seed the EE setpoint (used at startup after the first FK).
    void setTarget(double x, double y) {
        target_x_ = x;
        target_y_ = y;
    }

    // ---- Angular velocity (state display, absorbs :562-576) ----------------

    // Numerical differentiation + LPF of the body-frame attitude, for /albc_status.
    // Byte-identical to albc_controller.cpp:562-576 (uses the same DERIV_LPF_ALPHA).
    // Runs every tick regardless of mode, exactly as the source did.
    void updateAngularVel(double current_roll, double current_pitch,
                          double current_yaw, double dt) {
        double raw_ang_vel_roll  = (current_roll  - prev_roll_)  / dt;
        double raw_ang_vel_pitch = (current_pitch - prev_pitch_) / dt;
        double raw_ang_vel_yaw   = (current_yaw   - prev_yaw_)  / dt;

        angular_vel_roll_  = ORACLE_DERIV_LPF_ALPHA * raw_ang_vel_roll  + (1.0 - ORACLE_DERIV_LPF_ALPHA) * filtered_ang_vel_roll_;
        angular_vel_pitch_ = ORACLE_DERIV_LPF_ALPHA * raw_ang_vel_pitch + (1.0 - ORACLE_DERIV_LPF_ALPHA) * filtered_ang_vel_pitch_;
        angular_vel_yaw_   = ORACLE_DERIV_LPF_ALPHA * raw_ang_vel_yaw   + (1.0 - ORACLE_DERIV_LPF_ALPHA) * filtered_ang_vel_yaw_;

        filtered_ang_vel_roll_  = angular_vel_roll_;
        filtered_ang_vel_pitch_ = angular_vel_pitch_;
        filtered_ang_vel_yaw_   = angular_vel_yaw_;

        prev_roll_  = current_roll;
        prev_pitch_ = current_pitch;
        prev_yaw_   = current_yaw;
    }

    // ---- Feedback update (absorbs the main-loop block :598-640) -------------

    // Run one feedback tick for the TDC / PID / FIXED modes and update the EE
    // setpoint (target_x_, target_y_). Byte-identical to albc_controller.cpp:
    //   error (:599-600) -> integralStep (:605-612) -> dampedDerivative
    //   (:615-633) -> computeControlOutputOracle (:636) -> prev save (:639-640).
    //
    // `mode` is the CtrlMode code (CTRL_TDC/PID/FIXED/MANUAL from control_law.h);
    // callers pass static_cast<int>(control_mode). MANUAL must not reach here
    // (the source bypasses this block for MANUAL), but the oracle no-ops on it.
    void update(int mode, double current_roll, double current_pitch, double dt) {
        current_roll_  = current_roll;
        current_pitch_ = current_pitch;

        // Error
        error_roll_  = target_roll_  - current_roll_;
        error_pitch_ = target_pitch_ - current_pitch_;

        // Integral accumulation (freeze at setpoint + symmetric anti-windup clamp).
        // Gated on the error since 2026-08-12; identical to the old current-angle
        // gate whenever the target is 0. See the control_law.h header note.
        integral_roll_  = integralStep(integral_roll_,  error_roll_,
                                       ORACLE_LEVEL_THRESHOLD, ORACLE_INTEGRAL_MAX);
        integral_pitch_ = integralStep(integral_pitch_, error_pitch_,
                                       ORACLE_LEVEL_THRESHOLD, ORACLE_INTEGRAL_MAX);

        // Derivative: asymmetric gate on RAW + 1st-order LPF
        double derivative_roll  = dampedDerivative(error_roll_,  prev_error_roll_,
                                                   filtered_deriv_roll_,  dt,
                                                   ORACLE_DERIV_LPF_ALPHA, filtered_deriv_roll_);
        double derivative_pitch = dampedDerivative(error_pitch_, prev_error_pitch_,
                                                   filtered_deriv_pitch_, dt,
                                                   ORACLE_DERIV_LPF_ALPHA, filtered_deriv_pitch_);

        // Control law (4-mode switch)
        CtrlIn in = {};
        in.mode           = mode;
        in.current_roll   = current_roll_;
        in.current_pitch  = current_pitch_;
        in.error_roll     = error_roll_;
        in.error_pitch    = error_pitch_;
        in.integral_roll  = integral_roll_;
        in.integral_pitch = integral_pitch_;
        in.deriv_roll     = derivative_roll;
        in.deriv_pitch    = derivative_pitch;
        in.target_x       = target_x_;
        in.target_y       = target_y_;
        in.M_td           = M_td_;
        in.Kp_td          = Kp_td_;
        in.kp_roll        = kp_roll_;
        in.ki_roll        = ki_roll_;
        in.kd_roll        = kd_roll_;
        in.kp_pitch       = kp_pitch_;
        in.ki_pitch       = ki_pitch_;
        in.kd_pitch       = kd_pitch_;

        CtrlOut out = computeControlOutputOracle(in);
        target_x_ = out.target_x;
        target_y_ = out.target_y;

        // Store previous
        prev_error_roll_  = error_roll_;
        prev_error_pitch_ = error_pitch_;
    }

    // ---- Mode-change reset (absorbs :536-553) ------------------------------

    // Reset feedback state for a clean TDC/PID transition: seed the setpoint to
    // the current EE position, zero the integrals, and clear the derivative /
    // angular-velocity memory so the next tick produces derivative = 0.
    // Byte-identical to albc_controller.cpp:537-553. The caller supplies the
    // current EE position (forwardKinematics result) and the current attitude.
    void resetForModeChange(double current_x, double current_y,
                            double current_roll, double current_pitch,
                            double current_yaw) {
        current_roll_  = current_roll;
        current_pitch_ = current_pitch;

        target_x_ = current_x;
        target_y_ = current_y;
        integral_roll_  = 0.0;
        integral_pitch_ = 0.0;

        prev_error_roll_      = error_roll_;
        prev_error_pitch_     = error_pitch_;
        filtered_deriv_roll_  = 0.0;
        filtered_deriv_pitch_ = 0.0;
        prev_roll_   = current_roll;
        prev_pitch_  = current_pitch;
        prev_yaw_    = current_yaw;
        filtered_ang_vel_roll_  = 0.0;
        filtered_ang_vel_pitch_ = 0.0;
        filtered_ang_vel_yaw_   = 0.0;
    }

    // ---- Getters (for IK call + /albc_status publish + dashboard) ----------

    double targetX() const { return target_x_; }
    double targetY() const { return target_y_; }

    double targetRoll()  const { return target_roll_; }
    double targetPitch() const { return target_pitch_; }
    double errorRoll()   const { return error_roll_; }
    double errorPitch()  const { return error_pitch_; }

    double angularVelRoll()  const { return angular_vel_roll_; }
    double angularVelPitch() const { return angular_vel_pitch_; }
    double angularVelYaw()   const { return angular_vel_yaw_; }

    // Active gains / multiplier — for the dashboard readout (:401-402).
    double gainMult() const { return gain_mult_; }
    double Mtd()      const { return M_td_; }
    double Kptd()     const { return Kp_td_; }

private:
    // Attitude state (former ControlState, albc_controller.cpp:113-124).
    double current_roll_ = 0.0, current_pitch_ = 0.0;
    double error_roll_ = 0.0, error_pitch_ = 0.0;
    double integral_roll_ = 0.0, integral_pitch_ = 0.0;
    double prev_error_roll_ = 0.0, prev_error_pitch_ = 0.0;
    double target_roll_ = 0.0, target_pitch_ = 0.0;
    double target_x_ = 0.0, target_y_ = 0.0;
    double filtered_deriv_roll_ = 0.0, filtered_deriv_pitch_ = 0.0;
    double prev_roll_ = 0.0, prev_pitch_ = 0.0, prev_yaw_ = 0.0;
    double angular_vel_roll_ = 0.0, angular_vel_pitch_ = 0.0, angular_vel_yaw_ = 0.0;
    double filtered_ang_vel_roll_ = 0.0, filtered_ang_vel_pitch_ = 0.0, filtered_ang_vel_yaw_ = 0.0;

    // Gains (former ControlGains, albc_controller.cpp:87-111).
    double M_td_base_ = 0.0, Kp_td_base_ = 0.0;
    double kp_roll_base_ = 0.0, ki_roll_base_ = 0.0, kd_roll_base_ = 0.0;
    double kp_pitch_base_ = 0.0, ki_pitch_base_ = 0.0, kd_pitch_base_ = 0.0;
    double M_td_ = 0.0, Kp_td_ = 0.0;
    double kp_roll_ = 0.0, ki_roll_ = 0.0, kd_roll_ = 0.0;
    double kp_pitch_ = 0.0, ki_pitch_ = 0.0, kd_pitch_ = 0.0;
    double gain_mult_ = 0.0;
};

}  // namespace albc

#endif  // ALBC_CONTROL_ATTITUDE_CONTROLLER_H
