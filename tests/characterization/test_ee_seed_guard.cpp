// Characterization test: measured-angle seeding + folded-singularity guard
//
// Pins the two claims albc_controller.cpp:main() now rests on when it seeds
// theta1/theta2 from /albc/joint_states instead of the yaml constants
// (2026-08-24). Both are pure geometry, so they are testable on a host with no
// ROS and no robot — which is the point: the seeding path itself is ROS
// plumbing, but the DECISIONS inside it are arithmetic.
//
//   CLAIM 1 (why wrapping the seed is safe).
//     /albc/joint_states carries the raw-CUMULATIVE measured angle
//     (joint_angle_command.cpp: js.position = meas.abs_meas), and J1 has been
//     tens of rad from zero in the field (-35.54 rad, 2026-08-17). The
//     controller is in the WRAPPED convention (StatusPublisher publishes
//     mapTo2Pi(theta)). Seeding with mapTo2Pi(position[i]) is therefore a
//     representation change, NOT a value change — everything downstream reaches
//     theta only through sin/cos. If this test fails, the seed is silently
//     moving the arm somewhere the cumulative value would not have.
//
//   CLAIM 2 (why the guard threshold is where it is).
//     L1 == L2, so the EE radius is 2*L1*|cos(theta2/2)| and collapses to 0 at
//     theta2 = pi — the parked pose. There the DLS damping
//     lambda = lambda_base*(1 - sqrt(|sin theta2|)) is at its MAXIMUM while the
//     Jacobian has lost the radial direction, so the step is pure damping and
//     the command freezes. Bench 2026-08-24: seed 180 deg held the command
//     bit-identical for 10 s with a 1.19 deg roll error standing.
//
// The bench radii quoted below (0.0161 frozen / 0.1252 responding) are
// CORROBORATION, not oracles — they were logged after the arm had already
// drifted, so they do not equal FK at the nominal seed. Nothing here asserts
// against them; the assertions use FK directly, which is exact.
//
// Build & run (local, no ROS):
//   c++ -std=c++11 -Wall -I. -I../../robot/albc_control/include \
//       test_ee_seed_guard.cpp -o /tmp/t && /tmp/t

#include "albc_control/albc_kinematics.h"
#include <cstdio>
#include <cmath>
#include <limits>

using albc::forwardKinematics;
using albc::mapTo2Pi;
using albc::L1;
using albc::L2;

// Must match albc_controller.cpp's min_ee_radius_m default and the yaml.
static const double MIN_EE_RADIUS_M = 0.05;

static int failures = 0, checks = 0;

static void expect_near(double got, double want, double tol, const char* desc)
{
    checks++;
    if (!std::isfinite(got) || std::fabs(got - want) > tol) {
        failures++;
        std::printf("FAIL [%s]: got %.12f want %.12f (tol %g)\n",
                    desc, got, want, tol);
    }
}

static void expect_true(bool cond, const char* desc)
{
    checks++;
    if (!cond) {
        failures++;
        std::printf("FAIL [%s]: expected true\n", desc);
    }
}

// The quantity albc_controller.cpp guards on, computed the same way it does.
static double ee_radius(double t1, double t2)
{
    double x = 0, y = 0;
    forwardKinematics(t1, t2, x, y);
    return std::sqrt(x * x + y * y);
}

int main()
{
    const double PI = M_PI;

    // ---------------------------------------------------------------------
    // CLAIM 1 — wrapping the seed does not move the arm.
    // Cumulative inputs spanning the field range, including the -35.54 rad J1
    // that the 2026-08-17 cable break left behind.
    // ---------------------------------------------------------------------
    {
        const double cumulative[][2] = {
            { -35.54,            PI          },  // J1 after the cable-break run
            {  12.0 * PI + 0.3,  PI - 0.2    },  // 6 turns + change
            { -4.0 * PI,         0.5 * PI    },
            {  0.0,              4.0 * PI + 1.0 },
            {  1.5,              2.0         },  // already inside [0, 2pi)
        };
        const int n = sizeof(cumulative) / sizeof(cumulative[0]);
        for (int i = 0; i < n; ++i) {
            double t1 = cumulative[i][0], t2 = cumulative[i][1];
            double xc = 0, yc = 0, xw = 0, yw = 0;
            forwardKinematics(t1, t2, xc, yc);
            forwardKinematics(mapTo2Pi(t1), mapTo2Pi(t2), xw, yw);
            // 1e-9 absolute: fmod on a value ~36 rad loses a few ulps, and the
            // guard compares a radius of ~0.1 m, so this is 8 orders of margin.
            expect_near(xw, xc, 1e-9, "wrap-invariant FK x");
            expect_near(yw, yc, 1e-9, "wrap-invariant FK y");
        }
        // The wrapped seed also lands in the range StatusPublisher publishes,
        // so the first command byte-matches the driver's prev_commanded
        // baseline (joint_angle_command.cpp seeds it with wrapTo2Pi too).
        expect_true(mapTo2Pi(-35.54) >= 0.0 && mapTo2Pi(-35.54) < 2.0 * PI,
                    "wrapped seed inside [0, 2pi)");
    }

    // ---------------------------------------------------------------------
    // CLAIM 2a — the EE radius is 2*L1*|cos(theta2/2)| and does not depend on
    // theta1. That independence is what lets the guard test theta2 alone.
    // ---------------------------------------------------------------------
    {
        expect_near(L1, L2, 1e-12, "L1 == L2 (the closed form assumes it)");
        const double t2s[] = { 0.0, 0.5, 1.0, 2.0, PI - 0.05, PI, PI + 0.4, 5.0 };
        const double t1s[] = { -3.0, 0.0, 0.7, 2.2, 6.0 };
        for (int i = 0; i < 8; ++i) {
            double want = 2.0 * L1 * std::fabs(std::cos(t2s[i] / 2.0));
            for (int j = 0; j < 5; ++j) {
                expect_near(ee_radius(t1s[j], t2s[i]), want, 1e-12,
                            "radius == 2*L1*|cos(t2/2)|, theta1-independent");
            }
        }
    }

    // ---------------------------------------------------------------------
    // CLAIM 2b — the guard fires exactly where the controller deadlocks.
    // ---------------------------------------------------------------------
    {
        // The parked pose. Radius is 0, so the guard MUST refuse.
        expect_near(ee_radius(1.0, PI), 0.0, 1e-15, "radius at the fold is 0");
        expect_true(ee_radius(1.0, PI) < MIN_EE_RADIUS_M, "guard fires at theta2 = pi");

        // 180.44 deg — the pose the arm was actually parked at on 2026-08-24.
        double parked = 180.44 * PI / 180.0;
        expect_true(ee_radius(1.0, parked) < MIN_EE_RADIUS_M,
                    "guard fires at the measured parked pose (180.44 deg)");

        // The damping-dominance point the 0.05 default was derived from:
        // sigma_min ~ L2*|sin t2| equals lambda = 0.15*(1 - sqrt(|sin t2|)) at
        // |sin t2| ~ 0.29, i.e. theta2 ~ 163 deg. The threshold must sit INSIDE
        // that radius, so the guard refuses only where motion is truly dead and
        // not merely sluggish.
        double t2_damp = PI - std::asin(0.29);
        expect_true(ee_radius(0.0, t2_damp) > MIN_EE_RADIUS_M,
                    "0.05 m default is inside the damping-dominance radius");
        expect_near(ee_radius(0.0, t2_damp), 0.0682, 5e-4,
                    "damping-dominance radius ~ 0.068 m");

        // Bench seeds that responded must pass the guard.
        expect_true(ee_radius(0.0, 150.0 * PI / 180.0) > MIN_EE_RADIUS_M,
                    "guard passes theta2 = 150 deg (bench: responding)");
        expect_true(ee_radius(0.0, 120.0 * PI / 180.0) > MIN_EE_RADIUS_M,
                    "guard passes theta2 = 120 deg (bench: responding)");
        expect_true(ee_radius(0.0, 90.0 * PI / 180.0) > MIN_EE_RADIUS_M,
                    "guard passes theta2 = 90 deg");

        // The refusal band, stated as an angle so the operator knows how far to
        // move the arm: 2*L1*|cos(t2/2)| < 0.05  <=>  |t2 - 180| < 12.3 deg.
        double half = std::acos(MIN_EE_RADIUS_M / (2.0 * L1));   // rad
        double band_deg = 180.0 - 2.0 * half * 180.0 / PI;
        expect_near(band_deg, 12.31, 0.02, "refusal band is +-12.3 deg around 180");
        expect_true(ee_radius(0.0, (180.0 - 13.0) * PI / 180.0) > MIN_EE_RADIUS_M,
                    "13 deg off the fold passes");
        expect_true(ee_radius(0.0, (180.0 - 11.0) * PI / 180.0) < MIN_EE_RADIUS_M,
                    "11 deg off the fold still refuses");
    }

    // ---------------------------------------------------------------------
    // CLAIM 3 — the guard comparison must be NaN-safe.
    // A NaN joint angle propagates into FK and out as a NaN radius. Written the
    // obvious way (r < min) the guard WAVES IT THROUGH, because every ordered
    // comparison against NaN is false. albc_controller.cpp therefore writes it
    // as !(r >= min) AND rejects a non-finite seed before the kinematics.
    // ---------------------------------------------------------------------
    {
        const double nan_r = std::numeric_limits<double>::quiet_NaN();
        expect_true(!(nan_r < MIN_EE_RADIUS_M),
                    "the naive form (r < min) FAILS to refuse NaN — this is the trap");
        expect_true(!(nan_r >= MIN_EE_RADIUS_M),
                    "the shipped form !(r >= min) refuses NaN");
        // The same holds for the value that actually reaches the guard.
        expect_true(!(ee_radius(nan_r, PI / 2.0) >= MIN_EE_RADIUS_M),
                    "a NaN joint angle produces a radius the guard refuses");
        // Infinity is the other non-finite input the seed check rejects: FK of
        // an infinite angle is NaN (inf * cos(inf) is undefined), not a big radius.
        const double inf = std::numeric_limits<double>::infinity();
        expect_true(!(ee_radius(inf, PI / 2.0) >= MIN_EE_RADIUS_M),
                    "an infinite joint angle does not sneak past as a large radius");
        expect_true(!std::isfinite(inf) && !std::isfinite(nan_r),
                    "isfinite() is the seed-side check that rejects both");
    }

    std::printf("%s: %d checks, %d failures\n",
                failures ? "FAIL" : "PASS", checks, failures);
    return failures ? 1 : 0;
}
