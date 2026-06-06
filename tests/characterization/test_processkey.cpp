// Characterization test: teleop processKey (Jetson-side target update)
//
// Pins the CURRENT behavior of teleop.cpp's processKey() — how translated keys
// move the target setpoint (target.x/y/z).
// Source: teleop.cpp.
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

#include "processkey_oracle.h"
#include <cstdio>
#include <cmath>

static int failures = 0, checks = 0;

static void expect_near(double got, double want, const char* desc)
{
    checks++;
    if (std::fabs(got - want) > 1e-9) {
        failures++;
        std::printf("FAIL [%s]: got %.9f want %.9f\n", desc, got, want);
    }
}
int main()
{
    const double XY = 0.05, Z = 0.01;   // teleop steps (config defaults)

    // Each key applied to a fresh zero state.
    { PkState s = {0,0,0}; process_key('w', XY, Z, s); expect_near(s.x, +XY, "w: x += xy_step"); }
    { PkState s = {0,0,0}; process_key('s', XY, Z, s); expect_near(s.x, -XY, "s: x -= xy_step"); }
    { PkState s = {0,0,0}; process_key('d', XY, Z, s); expect_near(s.y, +XY, "d: y += xy_step"); }
    { PkState s = {0,0,0}; process_key('a', XY, Z, s); expect_near(s.y, -XY, "a: y -= xy_step"); }
    { PkState s = {0,0,0}; process_key('r', XY, Z, s); expect_near(s.z, -Z,  "r: z -= z_step (heave up)"); }
    { PkState s = {0,0,0}; process_key('f', XY, Z, s); expect_near(s.z, +Z,  "f: z += z_step (heave down)"); }

    // Unmapped key: no state change at all
    { PkState s = {1,2,3}; process_key('Z', XY, Z, s);
      expect_near(s.x, 1, "unmapped: x unchanged");
      expect_near(s.y, 2, "unmapped: y unchanged");
      expect_near(s.z, 3, "unmapped: z unchanged"); }

    // Negative-char guard (processKey returns early if ch < 0)
    { PkState s = {0,0,0}; process_key(-1, XY, Z, s);
      expect_near(s.x, 0, "ch<0: ignored"); }

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
