// Characterization test: teleop processKey (Jetson-side target update)
//
// Pins the CURRENT behavior of teleop_controller.h — how translated keys
// move the target setpoint (target.x/y/z).
// Source: teleop_controller.h.
//
//   w -> x += xy_step      s -> x -= xy_step
//   d -> y += xy_step      a -> y -= xy_step
//   r -> z -= z_step       f -> z += z_step
//
// Note the asymmetry pinned here: 'r' DECREMENTS z, 'f' INCREMENTS z
// (heave sign convention), and a/d / s map to the LOWER setpoint.
//
// Build & run (local, no ROS):
//   c++ -std=c++11 -I. tests/characterization/test_processkey.cpp -o /tmp/t && /tmp/t

#include "hero_agent/teleop_controller.h"
#include <cstdio>
#include <cmath>

static int failures = 0, checks = 0;

static void expect_near(double got, double want, const char* desc)
{
    checks++;
    if (!std::isfinite(got) || std::fabs(got - want) > 1e-9) {
        failures++;
        std::printf("FAIL [%s]: got %.9f want %.9f\n", desc, got, want);
    }
}
int main()
{
    const double XY = 0.05, Z = 0.01;   // teleop steps (config defaults)

    // Each key applied to a fresh zero state.
    { hero::TeleopController t(XY, Z); t.apply('w'); expect_near(t.x(), +XY, "w: x += xy_step"); }
    { hero::TeleopController t(XY, Z); t.apply('s'); expect_near(t.x(), -XY, "s: x -= xy_step"); }
    { hero::TeleopController t(XY, Z); t.apply('d'); expect_near(t.y(), +XY, "d: y += xy_step"); }
    { hero::TeleopController t(XY, Z); t.apply('a'); expect_near(t.y(), -XY, "a: y -= xy_step"); }
    { hero::TeleopController t(XY, Z); t.apply('r'); expect_near(t.z(), -Z,  "r: z -= z_step (heave up)"); }
    { hero::TeleopController t(XY, Z); t.apply('f'); expect_near(t.z(), +Z,  "f: z += z_step (heave down)"); }
    // Unmapped key: reports "no change" AND leaves every axis untouched.
    { hero::TeleopController t(XY, Z);
      expect_near(t.apply('z') ? 1 : 0, 0, "unknown key ignored");
      expect_near(t.x(), 0, "unmapped: x unchanged");
      expect_near(t.y(), 0, "unmapped: y unchanged");
      expect_near(t.z(), 0, "unmapped: z unchanged"); }

    // Negative char (EOF / high-bit byte) falls through the same default arm.
    { hero::TeleopController t(XY, Z);
      expect_near(t.apply((char)-1) ? 1 : 0, 0, "ch<0: ignored");
      expect_near(t.z(), 0, "ch<0: z unchanged"); }

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
