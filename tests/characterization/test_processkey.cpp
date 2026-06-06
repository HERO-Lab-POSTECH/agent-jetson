// Characterization test: teleop processKey (Jetson-side target update)
//
// Pins the CURRENT behavior of teleop.cpp's processKey() — how translated keys
// move the target setpoint (target.x/y/z) and toggle the lawnmower flag.
// Source: teleop.cpp:82-107.
//
//   w -> x += xy_step      s -> x -= xy_step
//   d -> y += xy_step      a -> y -= xy_step
//   r -> z -= z_step       f -> z += z_step
//   o -> lawnmower = 0     p -> lawnmower = 1 (and zero sway/surge counts)
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
static void expect_eq(int got, int want, const char* desc)
{
    checks++;
    if (got != want) { failures++; std::printf("FAIL [%s]: got %d want %d\n", desc, got, want); }
}

int main()
{
    const double XY = 0.05, Z = 0.01;   // teleop steps (config defaults)

    // Each key applied to a fresh zero state.
    { PkState s = {0,0,0,0,0,0}; process_key('w', XY, Z, s); expect_near(s.x, +XY, "w: x += xy_step"); }
    { PkState s = {0,0,0,0,0,0}; process_key('s', XY, Z, s); expect_near(s.x, -XY, "s: x -= xy_step"); }
    { PkState s = {0,0,0,0,0,0}; process_key('d', XY, Z, s); expect_near(s.y, +XY, "d: y += xy_step"); }
    { PkState s = {0,0,0,0,0,0}; process_key('a', XY, Z, s); expect_near(s.y, -XY, "a: y -= xy_step"); }
    { PkState s = {0,0,0,0,0,0}; process_key('r', XY, Z, s); expect_near(s.z, -Z,  "r: z -= z_step (heave up)"); }
    { PkState s = {0,0,0,0,0,0}; process_key('f', XY, Z, s); expect_near(s.z, +Z,  "f: z += z_step (heave down)"); }

    // Lawnmower control keys
    { PkState s = {0,0,0,0,0,0}; process_key('p', XY, Z, s);
      expect_eq(s.lawnmower, 1, "p: lawnmower = 1");
      expect_eq(s.sway_count, 0, "p: sway_count reset");
      expect_eq(s.surge_count, 0, "p: surge_count reset"); }
    { PkState s = {0,0,0,0,1,1}; s.lawnmower = 1; process_key('o', XY, Z, s);
      expect_eq(s.lawnmower, 0, "o: lawnmower = 0"); }

    // Unmapped key: no state change at all
    { PkState s = {1,2,3,1,5,6}; process_key('Z', XY, Z, s);
      expect_near(s.x, 1, "unmapped: x unchanged");
      expect_near(s.y, 2, "unmapped: y unchanged");
      expect_near(s.z, 3, "unmapped: z unchanged");
      expect_eq(s.lawnmower, 1, "unmapped: lawnmower unchanged"); }

    // Negative-char guard (processKey returns early if ch < 0)
    { PkState s = {0,0,0,0,0,0}; process_key(-1, XY, Z, s);
      expect_near(s.x, 0, "ch<0: ignored"); }

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
