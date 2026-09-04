// Regression test: joint command baseline arithmetic (joint_unwrap.h)
//
// This is not a hand-written scenario. The table below is the 2026-08-13 field
// session, replayed: every (meas0, cmd0) pair is the measured joint1 angle and
// the first published joint1 command read out of that run's rosbag. That session
// is the one that ended with the J1->J2 daisy-chain cable torn apart.
//
// What is checked, in order:
//   1. unwrapNearest unit behaviour, including the (-pi, pi] boundary convention
//      and agreement with the old single-step rule wherever |delta| < pi.
//   2. THE OLD BUG, pinned. Replaying the historic init (wrapped baseline) with
//      the historic single-step unwrap must still reproduce +1, -1 and -3 turns
//      of excess on exactly the three runs where it happened, and 0.000 on the
//      other eleven. If someone "simplifies" the oracle and these move, the test
//      says so.
//   3. THE FIX. Same replay with unwrapNearest gives 0.000 excess on every run
//      where the policy was not truncated.
//   4. The one run where it does NOT, and why -- 22-34-34 seeded at -21.953 rad,
//      outside the old policy-side +-6pi clip, so the clip itself turned the
//      first command into a 3.2 rad step. No unwrap rule can absorb a step
//      larger than pi. This is the case that forced the clip removal, and both
//      halves of that claim are pinned here.
//   5. overGuard at the operator's 3-turn ceiling (2026-08-17 decision: 3 turns
//      is the last line, 4 turns is dangerous).
//
// "excess" = (driver's first servo goal) - (what the controller asked for).
// Zero means the driver went exactly where it was told.
//
// Build & run (local, no ROS):
//   tests/characterization/run.sh          # picks this up automatically

#include "albc_control/joint_unwrap.h"
#include <cstdio>
#include <cmath>

using albc::unwrapNearest;
using albc::unwrapSingleStep;
using albc::wrapTo2Pi;
using albc::overGuard;

static const double TWO_PI = 2.0 * M_PI;

static int failures = 0, checks = 0;

static void expect_near(double got, double want, const char* desc)
{
    checks++;
    if (!std::isfinite(got) || std::fabs(got - want) > 1e-6) {
        failures++;
        std::printf("FAIL [%s]: got %.9f want %.9f\n", desc, got, want);
    }
}

static void expect_true(bool got, const char* desc)
{
    checks++;
    if (!got) {
        failures++;
        std::printf("FAIL [%s]: expected true\n", desc);
    }
}

static void expect_false(bool got, const char* desc)
{
    checks++;
    if (got) {
        failures++;
        std::printf("FAIL [%s]: expected false\n", desc);
    }
}

// Replay the driver's first command with a chosen unwrap rule.
//   init:  absolute_angle = meas0 ; prev_commanded = wrapTo2Pi(meas0)
//   first: absolute_angle += unwrap(cmd0 - prev_commanded)
//   excess = absolute_angle - cmd0
static double first_excess(double meas0, double cmd0, double (*unwrap)(double))
{
    double prev_commanded = wrapTo2Pi(meas0);
    double absolute_angle = meas0 + unwrap(cmd0 - prev_commanded);
    return absolute_angle - cmd0;
}

struct Run {
    const char* bag;      // rosbag basename (board clock; the board RTC is dead)
    double meas0;         // measured joint1 angle at the first command (rad)
    double cmd0;          // first joint1 command published (rad)
    double old_excess;    // turns of excess under the SHIPPED (buggy) rule
};

// joint1 only -- joint2 has no cable routing and no guard.
static const Run RUNS[] = {
    {"2026-08-11-12-43-33",  -0.008,  -0.051,  0.0},
    {"2026-08-11-12-52-36",  -0.144,  -0.150,  0.0},
    {"2026-08-11-13-06-56",   0.781,   0.690,  0.0},
    {"2026-08-12-20-51-04",   3.289,   2.265,  0.0},
    {"2026-08-12-21-46-38",   1.319,   0.976,  0.0},
    {"2026-08-12-22-15-00",  -4.709,  -4.629,  0.0},
    {"2026-08-12-22-16-41",  14.140,  13.040,  1.0},   // +1 turn
    {"2026-08-12-22-18-39",  -4.711,  -4.694,  0.0},
    {"2026-08-12-22-23-23",  -3.993,  -3.920,  0.0},
    {"2026-08-12-22-29-18",  -3.901,  -3.803,  0.0},
    {"2026-08-12-22-30-00",   4.909,   4.780,  0.0},
    {"2026-08-12-22-32-46",   2.187,   1.299,  0.0},
    {"2026-08-12-22-33-32",  -6.680,  -6.527, -1.0},   // -1 turn
    {"2026-08-12-22-34-34", -21.953, -18.755, -3.0},   // -3 turns; policy clip
};
static const int N_RUNS = static_cast<int>(sizeof(RUNS) / sizeof(RUNS[0]));

// The run whose first command was a truncated seed rather than an accumulator
// step. Indexed rather than name-matched so a table edit cannot silently
// de-target the special case.
static const int CLIPPED_RUN = 13;

int main()
{
    std::printf("-- 1. unwrapNearest unit behaviour --\n");

    expect_near(unwrapNearest(0.0), 0.0, "0 -> 0");
    expect_near(unwrapNearest(0.1), 0.1, "0.1 passes through");
    expect_near(unwrapNearest(-0.1), -0.1, "-0.1 passes through");
    // boundary convention, inherited from the shipped rule: +pi stays, -pi flips
    expect_near(unwrapNearest(M_PI), M_PI, "+pi stays +pi");
    expect_near(unwrapNearest(-M_PI), M_PI, "-pi maps to +pi");
    // whole turns vanish, however many
    for (int k = -4; k <= 4; ++k) {
        char d[80];
        std::sprintf(d, "k=%d turns + 0.1 folds to 0.1", k);
        expect_near(unwrapNearest(k * TWO_PI + 0.1), 0.1, d);
        std::sprintf(d, "k=%d turns exactly folds to 0", k);
        expect_near(unwrapNearest(k * TWO_PI), 0.0, d);
    }
    // agreement with the old rule inside the band it handled correctly
    for (int i = -30; i <= 30; ++i) {
        double d = i * (M_PI / 31.0);           // strictly inside (-pi, pi)
        char desc[80];
        std::sprintf(desc, "agrees with single-step at %.4f", d);
        expect_near(unwrapNearest(d), unwrapSingleStep(d), desc);
    }
    // and where they diverge: 2+ turns apart is exactly what the old rule missed
    expect_near(unwrapSingleStep(11.466), 11.466 - TWO_PI, "old rule removes ONE turn");
    expect_near(unwrapNearest(11.466), 11.466 - 2.0 * TWO_PI, "new rule removes TWO");

    std::printf("-- 2. the shipped rule, replayed on the 2026-08-13 bags --\n");
    for (int i = 0; i < N_RUNS; ++i) {
        double got = first_excess(RUNS[i].meas0, RUNS[i].cmd0, unwrapSingleStep);
        char desc[160];
        std::sprintf(desc, "OLD %s excess = %.1f turns", RUNS[i].bag, RUNS[i].old_excess);
        expect_near(got, RUNS[i].old_excess * TWO_PI, desc);
    }
    // ... and the reason it went unnoticed: the defect is EXACTLY zero whenever
    // the arm starts inside one turn, which was 11 of these 14 runs.
    for (int i = 0; i < N_RUNS; ++i) {
        if (std::fabs(RUNS[i].meas0) < TWO_PI) {
            char desc[160];
            std::sprintf(desc, "OLD %s within 1 turn -> silent", RUNS[i].bag);
            expect_near(first_excess(RUNS[i].meas0, RUNS[i].cmd0, unwrapSingleStep),
                        0.0, desc);
        }
    }

    std::printf("-- 3. the fix, same replay --\n");
    for (int i = 0; i < N_RUNS; ++i) {
        if (i == CLIPPED_RUN) continue;         // handled in section 4
        char desc[160];
        std::sprintf(desc, "NEW %s excess = 0", RUNS[i].bag);
        expect_near(first_excess(RUNS[i].meas0, RUNS[i].cmd0, unwrapNearest), 0.0, desc);
    }
    // the seed case in general form: a policy that seeds its accumulator from the
    // measured angle publishes meas0 + (one delta step at most). For ANY starting
    // wind k, the first command must move the arm by that step and nothing else.
    for (int k = -6; k <= 6; ++k) {
        double meas0 = k * TWO_PI + 0.37;       // arbitrary offset within a turn
        double cmd0 = meas0 + 0.10;             // DELTA_SCALE * |action| = max step
        char desc[96];
        std::sprintf(desc, "seed at %d turns -> only the 0.10 step survives", k);
        expect_near(first_excess(meas0, cmd0, unwrapNearest), 0.0, desc);
    }

    std::printf("-- 4. the truncated seed (why the policy clip had to go) --\n");
    {
        const Run& r = RUNS[CLIPPED_RUN];
        // As recorded, with the clip in place, the fix alone still loses a turn:
        // cmd0 is 3.198 rad from meas0, and a step that large cannot survive any
        // fold into (-pi, pi].
        expect_near(first_excess(r.meas0, r.cmd0, unwrapNearest), -TWO_PI,
                    "clipped seed still costs 1 turn under the new rule");
        expect_true(std::fabs(r.cmd0 - r.meas0) > M_PI,
                    "the clipped first command really is more than pi away");
        // Without the clip the policy would have published meas0 + one delta
        // step, and then the fix gives exactly zero.
        expect_near(first_excess(r.meas0, r.meas0 + 0.10, unwrapNearest), 0.0,
                    "unclipped seed -> 0 excess");
        // Sanity on the historic number itself: -3 turns from a -21.953 start.
        expect_near(first_excess(r.meas0, r.cmd0, unwrapSingleStep), -3.0 * TWO_PI,
                    "shipped rule drove 3 turns past the request");
    }

    std::printf("-- 5. joint1 cable guard (3-turn ceiling) --\n");
    {
        const double ABORT = 6.0 * M_PI;        // ~joint1_abort_rad default
        const double COUNT = 4.0 * M_PI;        // training constraint, metered only
        expect_false(overGuard(0.0, ABORT), "0 is inside");
        expect_false(overGuard(2.9 * TWO_PI, ABORT), "2.9 turns is inside");
        expect_false(overGuard(-3.0 * TWO_PI, ABORT), "exactly 3 turns is not over");
        expect_true(overGuard(3.01 * TWO_PI, ABORT), "3.01 turns trips");
        expect_true(overGuard(-3.01 * TWO_PI, ABORT), "-3.01 turns trips");
        // the 2026-08-13 endpoint must trip, on either sign of the check
        expect_true(overGuard(-35.54, ABORT), "the -35.54 rad break point trips");
        // the metering band is strictly inside the abort band
        expect_true(COUNT < ABORT, "count threshold is inside the abort threshold");
        expect_true(overGuard(-35.54, COUNT), "the break point is also counted");
        // a non-positive limit disables the guard
        expect_false(overGuard(1e6, 0.0), "limit 0 disables");
        expect_false(overGuard(1e6, -1.0), "negative limit disables");
    }

    std::printf("\n%d checks, %d failures\n", checks, failures);
    if (failures == 0) {
        std::printf("PASS\n");
        return 0;
    }
    std::printf("FAIL\n");
    return 1;
}
