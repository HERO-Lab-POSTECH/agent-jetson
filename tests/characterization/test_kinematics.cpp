// Characterization test: albc_control kinematics (FK / Jacobian / angle wrap)
//
// Pins the CURRENT behavior of albc_control/albc_kinematics.h — the pure,
// ROS-free math the controller (albc_controller.cpp) calls directly:
//   forwardKinematics(), calculateJacobian(), mapTo2Pi(), mapTo360().
//
// This header is already ROS-free, so NO oracle extraction is needed — the test
// #includes the real header and self-pins "current implementation == golden".
// A handful of clean inputs are ALSO cross-checked against the closed-form
// equations so an oracle typo (wrong sign/constant) would still be caught.
//
// Geometry recap (2-DOF planar, L1 = L2 = 0.233 m):
//   FK:  x = L1 cos(t1) + L2 cos(t1+t2)
//        y = L1 sin(t1) + L2 sin(t1+t2)
//   J:   j11 = -L1 sin(t1) - L2 sin(t1+t2)   j12 = -L2 sin(t1+t2)
//        j21 =  L1 cos(t1) + L2 cos(t1+t2)   j22 =  L2 cos(t1+t2)
//   FK(90,90) = (-L2, L1) = (-0.233, 0.233)   FK(0,0) = (L1+L2, 0) = (0.466, 0)
//
// Build & run (local, no ROS):
//   c++ -std=c++11 -Wall -I. -I../../robot/albc_control/include \
//       tests/characterization/test_kinematics.cpp -o /tmp/t && /tmp/t

#include "albc_control/albc_kinematics.h"
#include <cstdio>
#include <cmath>

using albc::forwardKinematics;
using albc::calculateJacobian;
using albc::mapTo2Pi;
using albc::mapTo360;
using albc::L1;
using albc::L2;

static int failures = 0, checks = 0;

static void expect_near(double got, double want, const char* desc)
{
    checks++;
    if (std::fabs(got - want) > 1e-9) {
        failures++;
        std::printf("FAIL [%s]: got %.12f want %.12f\n", desc, got, want);
    }
}

// Self-pin one FK input against the closed-form formula (independent recompute
// from L1/L2 — the test's own arithmetic, NOT a call to the function under test).
static void check_fk(double t1, double t2, const char* tag)
{
    double x = 0, y = 0;
    forwardKinematics(t1, t2, x, y);
    double ex = L1 * std::cos(t1) + L2 * std::cos(t1 + t2);
    double ey = L1 * std::sin(t1) + L2 * std::sin(t1 + t2);
    expect_near(x, ex, tag);
    expect_near(y, ey, tag);
}

// Self-pin one Jacobian input against the closed-form formula.
static void check_jac(double t1, double t2, const char* tag)
{
    double j11 = 0, j12 = 0, j21 = 0, j22 = 0;
    calculateJacobian(t1, t2, j11, j12, j21, j22);
    double e11 = -L1 * std::sin(t1) - L2 * std::sin(t1 + t2);
    double e12 = -L2 * std::sin(t1 + t2);
    double e21 =  L1 * std::cos(t1) + L2 * std::cos(t1 + t2);
    double e22 =  L2 * std::cos(t1 + t2);
    expect_near(j11, e11, tag);
    expect_near(j12, e12, tag);
    expect_near(j21, e21, tag);
    expect_near(j22, e22, tag);
}

int main()
{
    const double PI = M_PI;
    const double HALF_PI = M_PI / 2.0;
    const double QUARTER_PI = M_PI / 4.0;

    // ---- forwardKinematics: independent closed-form cross-check (catches
    //      oracle typos) on clean and arbitrary inputs ----
    check_fk(0.0,        0.0,        "FK(0,0)");
    check_fk(HALF_PI,    HALF_PI,    "FK(90,90)");
    check_fk(QUARTER_PI, QUARTER_PI, "FK(45,45)");
    check_fk(HALF_PI,    0.0,        "FK(90,0)");
    check_fk(0.0,        HALF_PI,    "FK(0,90)");
    check_fk(0.3,        -0.7,       "FK(0.3,-0.7)");
    check_fk(-1.2,        2.5,       "FK(-1.2,2.5)");

    // ---- forwardKinematics: hard-pinned exact golden values (regression
    //      detector independent of the formula recompute above) ----
    {
        double x = 0, y = 0;
        forwardKinematics(HALF_PI, HALF_PI, x, y);
        expect_near(x, -L2,  "FK(90,90).x == -L2 (-0.233)");
        expect_near(y,  L1,  "FK(90,90).y ==  L1 ( 0.233)");
    }
    {
        double x = 0, y = 0;
        forwardKinematics(0.0, 0.0, x, y);
        expect_near(x, L1 + L2, "FK(0,0).x == L1+L2 (0.466)");
        expect_near(y, 0.0,     "FK(0,0).y == 0");
    }
    {
        // t1=90,t2=0: fully extended pointing +y -> (0, L1+L2)
        double x = 0, y = 0;
        forwardKinematics(HALF_PI, 0.0, x, y);
        expect_near(x, 0.0,     "FK(90,0).x == 0");
        expect_near(y, L1 + L2, "FK(90,0).y == L1+L2 (0.466)");
    }
    {
        // t1=0,t2=180: link2 folds back onto link1 -> (L1-L2, 0) = (0,0)
        double x = 0, y = 0;
        forwardKinematics(0.0, PI, x, y);
        expect_near(x, 0.0, "FK(0,180).x == L1-L2 (0)");
        expect_near(y, 0.0, "FK(0,180).y == 0");
    }

    // ---- calculateJacobian: independent closed-form cross-check ----
    check_jac(0.0,        0.0,        "J(0,0)");
    check_jac(HALF_PI,    HALF_PI,    "J(90,90)");
    check_jac(QUARTER_PI, QUARTER_PI, "J(45,45)");
    check_jac(HALF_PI,    0.0,        "J(90,0)");
    check_jac(0.3,       -0.7,        "J(0.3,-0.7)");
    check_jac(-1.2,       2.5,        "J(-1.2,2.5)");

    // ---- calculateJacobian: hard-pinned exact golden values ----
    {
        // J(0,0): sin terms 0, cos terms 1 -> [[0,0],[L1+L2, L2]]
        double j11 = 0, j12 = 0, j21 = 0, j22 = 0;
        calculateJacobian(0.0, 0.0, j11, j12, j21, j22);
        expect_near(j11, 0.0,     "J(0,0).j11 == 0");
        expect_near(j12, 0.0,     "J(0,0).j12 == 0");
        expect_near(j21, L1 + L2, "J(0,0).j21 == L1+L2 (0.466)");
        expect_near(j22, L2,      "J(0,0).j22 == L2 (0.233)");
    }
    {
        // J(90,90): t1=90 -> sin=1,cos=0 ; t1+t2=180 -> sin=0,cos=-1
        //   j11 = -L1*1 - L2*0 = -L1 ; j12 = -L2*0 = 0
        //   j21 =  L1*0 + L2*(-1) = -L2 ; j22 = L2*(-1) = -L2
        double j11 = 0, j12 = 0, j21 = 0, j22 = 0;
        calculateJacobian(HALF_PI, HALF_PI, j11, j12, j21, j22);
        expect_near(j11, -L1, "J(90,90).j11 == -L1 (-0.233)");
        expect_near(j12, 0.0, "J(90,90).j12 == 0");
        expect_near(j21, -L2, "J(90,90).j21 == -L2 (-0.233)");
        expect_near(j22, -L2, "J(90,90).j22 == -L2 (-0.233)");
    }

    // ---- mapTo2Pi: [0, 2*pi) wrap boundaries ----
    expect_near(mapTo2Pi(-PI),            PI,                 "mapTo2Pi(-pi) == pi");
    expect_near(mapTo2Pi(3.0 * PI),       PI,                 "mapTo2Pi(3pi) == pi");
    expect_near(mapTo2Pi(2.0 * PI),       0.0,                "mapTo2Pi(2pi) == 0");
    expect_near(mapTo2Pi(-0.1),           2.0 * PI - 0.1,     "mapTo2Pi(-0.1) == 2pi-0.1");
    expect_near(mapTo2Pi(0.0),            0.0,                "mapTo2Pi(0) == 0");
    expect_near(mapTo2Pi(PI),             PI,                 "mapTo2Pi(pi) == pi");
    expect_near(mapTo2Pi(2.0 * PI + 0.5), 0.5,                "mapTo2Pi(2pi+0.5) == 0.5");
    expect_near(mapTo2Pi(-2.0 * PI),      0.0,                "mapTo2Pi(-2pi) == 0");
    expect_near(mapTo2Pi(4.5 * PI),       0.5 * PI,           "mapTo2Pi(4.5pi) == 0.5pi");

    // ---- mapTo360: [0, 360) wrap boundaries (degree analog) ----
    expect_near(mapTo360(-180.0),  180.0,  "mapTo360(-180) == 180");
    expect_near(mapTo360(540.0),   180.0,  "mapTo360(540) == 180");
    expect_near(mapTo360(360.0),   0.0,    "mapTo360(360) == 0");
    expect_near(mapTo360(-0.1),    359.9,  "mapTo360(-0.1) == 359.9");
    expect_near(mapTo360(0.0),     0.0,    "mapTo360(0) == 0");
    expect_near(mapTo360(180.0),   180.0,  "mapTo360(180) == 180");
    expect_near(mapTo360(360.5),   0.5,    "mapTo360(360.5) == 0.5");
    expect_near(mapTo360(-360.0),  0.0,    "mapTo360(-360) == 0");
    expect_near(mapTo360(810.0),   90.0,   "mapTo360(810) == 90");

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
