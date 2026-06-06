// Production header: InverseKinematics — DLS IK solver for albc_controller.
//
// Encapsulates the two byte-identical IK blocks the controller ran inline:
//   - MANUAL POSITION   (albc_controller.cpp:642-662, pre-refactor lines)
//   - feedback TDC/PID/FIXED (albc_controller.cpp:711-734, pre-refactor lines)
// Both blocks were: radial saturation -> delta/delta_norm -> dynamic_iterations
// (delta_norm > IK_DELTA_THRESHOLD ? num_iterations : IK_REDUCED_ITERATIONS) ->
// for(i<iter){ FK; delta; updateJointAngles } -> final FK. solveIK() reproduces
// that sequence 1:1; only "where it is computed" changes, never "how many / when".
//
// The single IK step delegates to dls_oracle::updateJointAnglesOracle (the Task-1
// promoted oracle in dls_ik.h) — the byte-identical transcription of the original
// global updateJointAngles(). step() passes the class-held learning_rate /
// lambda_base, exactly the values the global ik_cfg used to hold.
//
// IK_DELTA_THRESHOLD / IK_REDUCED_ITERATIONS were file-scope constexpr at
// albc_controller.cpp:80-81; transcribed here verbatim (values preserved).

#ifndef ALBC_CONTROL_INVERSE_KINEMATICS_H
#define ALBC_CONTROL_INVERSE_KINEMATICS_H

#include <cmath>
#include "albc_control/albc_kinematics.h"  // forwardKinematics, SAFE_ARM_LENGTH
#include "albc_control/dls_ik.h"            // dls_oracle::updateJointAnglesOracle

namespace albc {

class InverseKinematics {
public:
    // Transcribed from albc_controller.cpp:80-81 (file-scope constexpr there).
    static constexpr double IK_DELTA_THRESHOLD    = 0.01;
    static constexpr int    IK_REDUCED_ITERATIONS = 500;

    // Absorbs the former IKConfig struct (albc_controller.cpp:128-132).
    void setConfig(double lr, double lambda, int iters) {
        learning_rate_  = lr;
        lambda_base_    = lambda;
        num_iterations_ = iters;
    }

    // Single DLS IK step. Delegates to the Task-1 oracle so the math stays the
    // byte-identical transcription of the original global updateJointAngles().
    //   theta1, theta2 : current joint angles [rad], mutated in place (in/out).
    //   delta_x, delta_y : Cartesian error fed to the IK step.
    void step(double& theta1, double& theta2,
              double delta_x, double delta_y) const {
        dls_oracle::updateJointAnglesOracle(theta1, theta2, delta_x, delta_y,
                                            learning_rate_, lambda_base_);
    }

    // Full IK solve: radial saturation + iterative DLS descent + final FK.
    // Reproduces albc_controller.cpp:642-662 / :711-734 verbatim.
    //   target_x, target_y : desired EE position [m] (in/out — saturated in place).
    //   theta1, theta2     : joint angles [rad] (in/out — converged by the loop).
    //   current_x, current_y : EE position [m] (in/out — left at the final FK).
    //   target_length      : OUT — radius used by the saturation check (dashboard).
    void solveIK(double& target_x, double& target_y,
                 double& theta1, double& theta2,
                 double& current_x, double& current_y,
                 double& target_length) const {
        // Radial workspace saturation
        target_length = std::sqrt(target_x * target_x + target_y * target_y);
        if (target_length >= SAFE_ARM_LENGTH) {
            target_x = target_x / target_length * SAFE_ARM_LENGTH;
            target_y = target_y / target_length * SAFE_ARM_LENGTH;
        }

        // Inverse kinematics
        double delta_x = target_x - current_x;
        double delta_y = target_y - current_y;
        double delta_norm = std::sqrt(delta_x * delta_x + delta_y * delta_y);

        int dynamic_iterations = (delta_norm > IK_DELTA_THRESHOLD)
                                 ? num_iterations_ : IK_REDUCED_ITERATIONS;

        for (int i = 0; i < dynamic_iterations; i++) {
            forwardKinematics(theta1, theta2, current_x, current_y);
            delta_x = target_x - current_x;
            delta_y = target_y - current_y;
            step(theta1, theta2, delta_x, delta_y);
        }

        // Final FK for display
        forwardKinematics(theta1, theta2, current_x, current_y);
    }

private:
    double learning_rate_  = 0.0;
    double lambda_base_    = 0.0;
    int    num_iterations_ = 0;
};

}  // namespace albc

#endif  // ALBC_CONTROL_INVERSE_KINEMATICS_H
