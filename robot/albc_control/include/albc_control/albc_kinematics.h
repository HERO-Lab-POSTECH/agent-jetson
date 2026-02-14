#ifndef ALBC_KINEMATICS_H
#define ALBC_KINEMATICS_H

#include <cmath>

// Unit conversion macros (parenthesized for safe macro expansion)
#define DEG2RAD(x) ((M_PI / 180.0) * (x))
#define RAD2DEG(x) ((x) / (M_PI / 180.0))

namespace albc {

// Physical constants for the 2-DOF ALBC manipulator
constexpr double L1 = 0.233;  // Link 1 length [m]
constexpr double L2 = 0.233;  // Link 2 length [m]
constexpr double Fb = 13.0;   // Buoyancy force [N]
constexpr double MAX_ARM_LENGTH  = L1 + L2;          // 0.466 m
constexpr double SAFE_ARM_LENGTH = MAX_ARM_LENGTH * 0.95;  // Singularity margin

/**
 * @brief Computes the forward kinematics for a 2-DOF planar manipulator.
 */
inline void forwardKinematics(double theta1, double theta2, double& x, double& y) {
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
}

/**
 * @brief Calculates the 2x2 Jacobian matrix for the 2-DOF manipulator.
 */
inline void calculateJacobian(double theta1, double theta2,
                               double& j11, double& j12,
                               double& j21, double& j22) {
    j11 = -L1 * sin(theta1) - L2 * sin(theta1 + theta2);
    j12 = -L2 * sin(theta1 + theta2);
    j21 =  L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    j22 =  L2 * cos(theta1 + theta2);
}

/**
 * @brief Maps a radian angle to the range [0, 2*pi).
 */
inline double mapTo2Pi(double angle) {
    double mapped = fmod(angle, 2.0 * M_PI);
    if (mapped < 0.0) mapped += 2.0 * M_PI;
    return mapped;
}

/**
 * @brief Maps a degree angle to the range [0, 360).
 */
inline double mapTo360(double angle) {
    double mapped = fmod(angle, 360.0);
    if (mapped < 0.0) mapped += 360.0;
    return mapped;
}

}  // namespace albc

#endif  // ALBC_KINEMATICS_H
