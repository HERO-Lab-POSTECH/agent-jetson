// Characterization test: teleop target accumulation (Jetson-side setpoint update)
//
// Pins the CURRENT behavior of teleop_controller.h — how a translated key moves
// the target setpoint (target.z).
// Source: teleop_controller.h.
//
//   r -> z -= z_step   (heave up)
//   f -> z += z_step   (heave down)
//
// Only 'r' and 'f' can ever reach apply(): KEYMAP gives w/s/a/d translated=0 and
// forwards them to the firmware as fw_char instead, so the jog keys never touch
// this accumulator. The xy branch that used to live here was removed with the
// x/y members it fed; this test pins that they are gone by asserting apply('w')
// reports "no change".
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
    const double Z = 0.01;   // teleop z step (config default)

    // Each key applied to a fresh zero state.
    { hero::TeleopController t(Z); t.apply('r'); expect_near(t.z(), -Z, "r: z -= z_step (heave up)"); }
    { hero::TeleopController t(Z); t.apply('f'); expect_near(t.z(), +Z, "f: z += z_step (heave down)"); }

    // Accumulation, not assignment.
    { hero::TeleopController t(Z); t.apply('f'); t.apply('f');
      expect_near(t.z(), 2 * Z, "f twice: accumulates"); }

    // The jog keys KEYMAP never translates: reported as "no change", state untouched.
    { hero::TeleopController t(Z);
      expect_near(t.apply('w') ? 1 : 0, 0, "w: not a teleop action");
      expect_near(t.apply('s') ? 1 : 0, 0, "s: not a teleop action");
      expect_near(t.apply('a') ? 1 : 0, 0, "a: not a teleop action");
      expect_near(t.apply('d') ? 1 : 0, 0, "d: not a teleop action");
      expect_near(t.z(), 0, "jog keys leave z untouched"); }

    // Unmapped key and a negative char both fall through the same default arm.
    { hero::TeleopController t(Z);
      expect_near(t.apply('z') ? 1 : 0, 0, "unknown key ignored");
      expect_near(t.apply((char)-1) ? 1 : 0, 0, "ch<0: ignored");
      expect_near(t.z(), 0, "ignored keys leave z untouched"); }

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
