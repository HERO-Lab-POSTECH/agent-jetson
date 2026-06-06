// Characterization test: lawnmower survey FSM
//
// Pins the CURRENT behavior of lawnmower_survey.cpp's executeLawnmowerSurvey().
// The redesign must preserve the exact state-transition sequence and target
// deltas, or this test fails.
//
// The FSM advances target.y / target.x in discrete steps gated by a 0..9
// `count` cycle (the 100Hz loop's modulo-10 counter in agent_command.cpp:93).
// State machine (lawnmower_survey.cpp:7-49):
//   1 = sway +  : every count==0 -> y += move_dis, sway_count++
//                 at count==9 with sway_count > sway_num -> go state 2
//   2 = surge   : every count==0 -> x += move_dis, surge_count++
//                 at count==9 with surge_count > surge_num -> go state 3
//   3 = sway -  : every count==0 -> y -= move_dis ... -> go state 4
//   4 = surge   : every count==0 -> x += move_dis ... -> go state 0 (done)
//
// Build & run (local, no ROS):
//   c++ -std=c++11 -I. tests/characterization/test_lawnmower_fsm.cpp -o /tmp/t && /tmp/t

#include "lawnmower_oracle.h"
#include <cstdio>
#include <cmath>

static int failures = 0;
static int checks = 0;

static void expect_near(double got, double want, const char* desc)
{
    checks++;
    if (std::fabs(got - want) > 1e-6) {
        failures++;
        std::printf("FAIL [%s]: got %.6f want %.6f\n", desc, got, want);
    }
}

static void expect_eq(int got, int want, const char* desc)
{
    checks++;
    if (got != want) {
        failures++;
        std::printf("FAIL [%s]: got %d want %d\n", desc, got, want);
    }
}

int main()
{
    // Small thresholds so the full 1->2->3->4->0 cycle runs in few iterations.
    // (Real defaults: sway_num=300, surge_num=50; behavior identical, just longer.)
    LawnmowerState lm;
    lm.move_dis = 0.01f;
    lm.sway_num = 2;     // leg ends after sway_count exceeds 2
    lm.surge_num = 1;    // leg ends after surge_count exceeds 1

    Target tgt = {0.0, 0.0, 0.0};
    int state = 1;       // FSM starts at state 1 when lawnmower activated

    // Drive the FSM through many 0..9 count cycles, recording transitions.
    // One "tick" = one count value. count cycles 0,1,...,9,0,1,...
    int prev_state = state;
    int transitions[8];   // record state values at each change
    int n_trans = 0;
    transitions[n_trans++] = state;

    int count = 0;
    for (int tick = 0; tick < 400 && state != 0; ++tick) {
        lawnmower_step(state, count, lm, tgt);
        if (state != prev_state) {
            if (n_trans < 8) transitions[n_trans++] = state;
            prev_state = state;
        }
        count = (count + 1) % 10;
    }

    // ── Transition sequence must be exactly 1 -> 2 -> 3 -> 4 -> 0 ──
    expect_eq(n_trans, 5, "exactly 5 states in sequence (1,2,3,4,0)");
    expect_eq(transitions[0], 1, "starts in state 1 (sway+)");
    expect_eq(transitions[1], 2, "state 1 -> 2 (surge)");
    expect_eq(transitions[2], 3, "state 2 -> 3 (sway-)");
    expect_eq(transitions[3], 4, "state 3 -> 4 (surge)");
    expect_eq(transitions[4], 0, "state 4 -> 0 (done)");

    // ── Final displacement: pinned to hand-computed constants ──
    // Derived from the spec, NOT from the oracle (independent check):
    //   sway+  leg: (sway_num+1) increments of +move_dis = 3 * 0.01 = +0.03 y
    //   surge  leg: (surge_num+1) increments of +move_dis = 2 * 0.01 = +0.02 x
    //   sway-  leg: (sway_num+1) decrements of -move_dis = 3 * 0.01 = -0.03 y
    //   surge  leg: (surge_num+1) increments of +move_dis = 2 * 0.01 = +0.02 x
    // Net: x = 0.02 + 0.02 = 0.04 ; y = +0.03 - 0.03 = 0.00
    expect_near(tgt.x, 0.04, "final target.x = two surge legs = 0.04");
    expect_near(tgt.y, 0.00, "final target.y = sway+ cancels sway- = 0.00");

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
