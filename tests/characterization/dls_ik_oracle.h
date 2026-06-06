// Golden oracle: albc_controller updateJointAngles (ROS-free extraction)
//
// 1:1 transcription of albc_controller.cpp:155-199 (the Damped-Least-Squares
// inverse-kinematics single step). The ROS plumbing is dropped because it does
// NOT affect the math:
//   - the ROS_WARN_THROTTLE(...) call at .cpp:171 only logs; it mutates no state.
// The two IK_cfg globals the original reads (ik_cfg.lambda_base at .cpp:156 and
// ik_cfg.learning_rate at .cpp:186-187) are passed in explicitly as
// `lambda_base` and `lr`. Everything else — variable names, signs, constants,
// operation order — is byte-identical to the source. The redesign must keep
// updateJointAngles() byte-identical to this.
//
// Reuses albc::calculateJacobian from the (pure, ROS-free) kinematics header,
// exactly as the controller does via `using namespace albc;` (.cpp:17, :162).
//
// Pinned constants (do NOT "fix" without intent), transcribed from
// albc_controller.cpp:74 and :79:
//   DET_EPSILON          = 1e-6   (near-singular determinant floor)
//   UPDATE_ANGLE_EPSILON = 1e-4   (clamped update magnitude when |Δθ| > π)

#ifndef DLS_IK_ORACLE_H
#define DLS_IK_ORACLE_H

#include <cmath>
#include "albc_control/albc_kinematics.h"  // albc::L1, albc::L2, calculateJacobian

namespace dls_oracle {

// Transcribed from albc_controller.cpp:74, :79 (file-scope constexpr there).
static constexpr double DET_EPSILON          = 1e-6;
static constexpr double UPDATE_ANGLE_EPSILON = 1e-4;

// Pure transcription of albc_controller.cpp:155-199.
//   theta1, theta2 : current joint angles [rad], mutated in place (in/out).
//   delta_x, delta_y : Cartesian error fed to the IK step.
//   lr             : ik_cfg.learning_rate  (was a global at .cpp:186-187).
//   lambda_base    : ik_cfg.lambda_base    (was a global at .cpp:156).
inline void updateJointAnglesOracle(double& theta1, double& theta2,
                                    double delta_x, double delta_y,
                                    double lr, double lambda_base)
{
    using albc::L1;
    using albc::L2;
    using albc::calculateJacobian;

    double lambda = lambda_base * (
        1.0 - std::sqrt(std::abs(L1 * L2 * std::sin(theta2))) /
              std::sqrt(std::abs(L1 * L2))
    );

    double j11, j12, j21, j22;
    calculateJacobian(theta1, theta2, j11, j12, j21, j22);

    // J^T * J + lambda^2 * I
    double jtj_11 = j11 * j11 + j21 * j21 + lambda * lambda;
    double jtj_12 = j11 * j12 + j21 * j22;
    double jtj_22 = j12 * j12 + j22 * j22 + lambda * lambda;

    double det = jtj_11 * jtj_22 - jtj_12 * jtj_12;
    if (std::abs(det) < DET_EPSILON) {
        // ROS_WARN_THROTTLE(2.0, ...) dropped — logging only, no state change.
        det = (det >= 0.0) ? DET_EPSILON : -DET_EPSILON;
    }

    // Inverse of (J^T * J + lambda^2 * I)
    double inv11 =  jtj_22 / det;
    double inv12 = -jtj_12 / det;
    double inv22 =  jtj_11 / det;

    // Pseudo-inverse: (J^T J + lambda^2 I)^{-1} * J^T
    double j11_inv = inv11 * j11 + inv12 * j12;
    double j12_inv = inv11 * j21 + inv12 * j22;
    double j21_inv = inv12 * j11 + inv22 * j12;
    double j22_inv = inv12 * j21 + inv22 * j22;

    double update_angle1 = lr * (j11_inv * delta_x + j12_inv * delta_y);
    double update_angle2 = lr * (j21_inv * delta_x + j22_inv * delta_y);

    // Clamp large updates (near singularity)
    if (std::abs(update_angle1) > M_PI) {
        update_angle1 = (update_angle1 > 0) ? UPDATE_ANGLE_EPSILON : -UPDATE_ANGLE_EPSILON;
    }
    if (std::abs(update_angle2) > M_PI) {
        update_angle2 = (update_angle2 > 0) ? UPDATE_ANGLE_EPSILON : -UPDATE_ANGLE_EPSILON;
    }

    theta1 += update_angle1;
    theta2 += update_angle2;
}

}  // namespace dls_oracle

#endif  // DLS_IK_ORACLE_H
