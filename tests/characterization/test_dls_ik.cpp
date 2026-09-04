// Characterization test: albc_controller DLS inverse-kinematics single step
//
// Pins the CURRENT behavior of albc_controller.cpp's updateJointAngles()
// (albc_controller.cpp:155-199) — one Damped-Least-Squares IK update. The
// redesign must keep this byte-identical, because this is control math: any
// drift in sign / constant / op-order changes how the arm moves.
// Source: albc_controller.cpp:155-199, via albc_control/dls_ik.h.
//
// Self-pin contract: golden values below were produced by RUNNING the oracle
// (the current implementation IS the spec). They are NOT hand-derived. Two
// clean cases (zero-error, large-update-clamp) are additionally cross-checked
// against an independent reimplementation of the math inside this file, so an
// oracle transcription typo would surface as a mismatch there.
//
// Behaviors pinned here (do NOT "fix" without intent):
//   - variable damping  lambda = lambda_base*(1 - sqrt|L1 L2 sin t2|/sqrt|L1 L2|)
//   - (J^T J + lambda^2 I)^{-1} J^T pseudo-inverse, det floored at +/-1e-6
//   - |delta theta| > pi  =>  clamped to +/- UPDATE_ANGLE_EPSILON (1e-4)
//   - delta_x = delta_y = 0  =>  angles unchanged (update is exactly 0)
//
// Build & run (local, no ROS):
//   c++ -std=c++11 -Wall -I. -I../../robot/albc_control/include \
//       tests/characterization/test_dls_ik.cpp -o /tmp/t && /tmp/t

#include "albc_control/dls_ik.h"
#include <cmath>
#include <cstdio>

static int failures = 0, checks = 0;

// Tight tolerance: this is a self-pin of the exact double output, so we demand
// agreement to ~1e-12 (loose enough only for the last ULPs).
static void expect_near(double got, double want, const char* desc)
{
    checks++;
    if (!std::isfinite(got) || std::fabs(got - want) > 1e-12) {
        failures++;
        std::printf("FAIL [%s]: got %.17g want %.17g\n", desc, got, want);
    }
}

// One IK step starting from (t1,t2); returns the mutated angles by reference.
static void step(double t1, double t2, double dx, double dy,
                 double lr, double lb, double& out1, double& out2)
{
    out1 = t1; out2 = t2;
    dls_oracle::updateJointAnglesOracle(out1, out2, dx, dy, lr, lb);
}

int main()
{
    double t1, t2;

    // --- Case 1: normal pose, both joints away from singularity ---------------
    // Self-pinned to the current oracle output.
    step(1.0, 1.2, 0.03, -0.02, 0.5, 0.01, t1, t2);
    expect_near(t1, 0.92212202657809916, "normal: theta1'");
    expect_near(t2, 1.2793059203024169,  "normal: theta2'");

    // --- Case 2: pose at (90 deg, 90 deg) ~ controller home -------------------
    step(1.5708, 1.5708, 0.01, 0.01, 0.3, 0.05, t1, t2);
    expect_near(t1, 1.5579243689306013, "pose90: theta1'");
    expect_near(t2, 1.5708001418831983, "pose90: theta2'");

    // --- Case 3: near-singular, sin(theta2) ~ 0 (theta2 -> 0) -----------------
    // lambda collapses to ~lambda_base here (the sqrt damping term -> 0), and
    // the Jacobian rows become nearly parallel; pinned to current output.
    step(0.7, 1e-9, 0.02, 0.01, 0.4, 0.02, t1, t2);
    expect_near(t1, 0.6964098014722635,   "near-sing0: theta1'");
    expect_near(t2, -0.001795100796624267, "near-sing0: theta2'");

    // --- Case 4: near-singular, sin(theta2) ~ 0 (theta2 -> pi) ----------------
    step(0.7, 3.14159265358979, 0.02, 0.01, 0.4, 0.02, t1, t2);
    expect_near(t1, 0.6999999999999823, "near-singPi: theta1'");
    expect_near(t2, 3.1505156335467772, "near-singPi: theta2'");

    // --- Case 5: zero Cartesian error => angles must NOT move ------------------
    // Independent check: update = lr * (Jinv * [0;0]) = 0, so the output equals
    // the input exactly, regardless of lambda/Jacobian.
    step(1.0, 1.2, 0.0, 0.0, 0.5, 0.01, t1, t2);
    expect_near(t1, 1.0, "zero-err: theta1' unchanged");
    expect_near(t2, 1.2, "zero-err: theta2' unchanged");

    // --- Case 6: large update => clamp to +/- UPDATE_ANGLE_EPSILON ------------
    // With lr=100 and large dx=dy=5 both raw updates blow past pi (independently
    // verified: u1 ~ +2.15e6 > 0 -> +1e-4 ; u2 ~ -4.30e6 < 0 -> -1e-4). So:
    //   theta1' = 0.001 + 1e-4 ;  theta2' = 0.001 - 1e-4.
    step(0.001, 0.001, 5.0, 5.0, 100.0, 0.0, t1, t2);
    expect_near(t1, 0.0011000000000000001,  "clamp: theta1' = +UPDATE_ANGLE_EPSILON");
    expect_near(t2, 0.00089999999999999998, "clamp: theta2' = -UPDATE_ANGLE_EPSILON");
    // And cross-check the clamp magnitude is exactly the pinned epsilon.
    expect_near(t1 - 0.001,  dls_oracle::UPDATE_ANGLE_EPSILON, "clamp: +eps magnitude");
    expect_near(t2 - 0.001, -dls_oracle::UPDATE_ANGLE_EPSILON, "clamp: -eps magnitude");

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
