// Characterization test: albc_controller feedback math (ROS-free)
//
// Pins the CURRENT behavior of albc_controller.cpp:674-702 — the asymmetric
// damping gate + integral anti-windup (freeze + clamp) + derivative LPF.
// Source of truth: albc_control/feedback_filters.h (byte-identical to the source).
//
// Strategy (self-pin): the test calls the oracle functions and EXPECTs their
// output. "Current implementation == correct". For the cleanest inputs an
// INDEPENDENT hand-computation is also asserted, so an oracle typo would be
// caught (the two must agree).
//
// Build & run (local, no ROS):
//   c++ -std=c++11 -Wall -I. test_damping_integral.cpp -o /tmp/t && /tmp/t

#include "albc_control/feedback_filters.h"
#include <cstdio>
#include <cmath>

static int failures = 0, checks = 0;

static void expect_near(double got, double want, const char* desc)
{
    checks++;
    if (std::fabs(got - want) > 1e-9) {
        failures++;
        std::printf("FAIL [%s]: got %.12f want %.12f\n", desc, got, want);
    }
}

int main()
{
    const double TH    = ORACLE_LEVEL_THRESHOLD; // 0.01745
    const double IMAX  = ORACLE_INTEGRAL_MAX;    // 100.0
    const double ALPHA = ORACLE_DERIV_LPF_ALPHA; // 0.2

    // ---------------------------------------------------------------------
    // 1. INTEGRAL — accumulation while NOT level (|angle| >= threshold).
    // ---------------------------------------------------------------------
    // Fresh integral 0, error +2, angle well above threshold -> += error.
    {
        double i = integralStep(0.0, 2.0, 0.5, TH, IMAX);
        expect_near(i, 2.0, "integral: accumulate (0 + 2)");          // hand: 0+2
    }
    // Repeated accumulation: feed result back in.
    {
        double i = 0.0;
        i = integralStep(i, 1.5, 0.5, TH, IMAX); // 1.5
        i = integralStep(i, 1.5, 0.5, TH, IMAX); // 3.0
        i = integralStep(i, 1.5, 0.5, TH, IMAX); // 4.5
        expect_near(i, 4.5, "integral: 3x accumulate (1.5*3)");       // hand: 4.5
    }
    // Negative error accumulates negatively.
    {
        double i = integralStep(10.0, -4.0, 0.5, TH, IMAX);
        expect_near(i, 6.0, "integral: negative error (10 + -4)");    // hand: 6.0
    }

    // ---------------------------------------------------------------------
    // 2. INTEGRAL — FREEZE while level (|angle| < threshold): integral held.
    // ---------------------------------------------------------------------
    // angle 0 (< threshold): integral unchanged even with large error.
    {
        double i = integralStep(7.0, 999.0, 0.0, TH, IMAX);
        expect_near(i, 7.0, "integral: frozen at angle=0 (held)");    // hand: held -> 7.0
    }
    // angle just BELOW threshold: still frozen.
    {
        double below = TH - 1e-6;
        double i = integralStep(3.0, 5.0, below, TH, IMAX);
        expect_near(i, 3.0, "integral: frozen just below threshold"); // hand: held -> 3.0
    }
    // angle EXACTLY at threshold: accumulates (>= is inclusive).
    {
        double i = integralStep(3.0, 5.0, TH, TH, IMAX);
        expect_near(i, 8.0, "integral: accumulates AT threshold (>=)");// hand: 3+5
    }
    // negative angle below threshold magnitude: frozen (abs() used).
    {
        double i = integralStep(2.0, 5.0, -(TH - 1e-6), TH, IMAX);
        expect_near(i, 2.0, "integral: frozen at small negative angle");// hand: held
    }

    // ---------------------------------------------------------------------
    // 3. INTEGRAL — symmetric clamp at +/- INTEGRAL_MAX.
    // ---------------------------------------------------------------------
    // Upper clamp: starting near max, large +error saturates to +IMAX.
    {
        double i = integralStep(99.0, 50.0, 0.5, TH, IMAX);
        expect_near(i, IMAX, "integral: clamp to +INTEGRAL_MAX");     // hand: min(100, 149)=100
    }
    // Lower clamp: large -error saturates to -IMAX.
    {
        double i = integralStep(-99.0, -50.0, 0.5, TH, IMAX);
        expect_near(i, -IMAX, "integral: clamp to -INTEGRAL_MAX");    // hand: max(-100,-149)=-100
    }
    // Exactly at max boundary stays at max.
    {
        double i = integralStep(IMAX, 1.0, 0.5, TH, IMAX);
        expect_near(i, IMAX, "integral: stays at +INTEGRAL_MAX");     // hand: min(100,101)=100
    }
    // Just inside the band is NOT clamped.
    {
        double i = integralStep(50.0, 10.0, 0.5, TH, IMAX);
        expect_near(i, 60.0, "integral: no clamp inside band");       // hand: 60.0
    }

    // ---------------------------------------------------------------------
    // 4. DAMPING GATE — sign cases (gate on RAW, before LPF).
    //    raw_deriv = (error - prev_error)/dt. error * raw_deriv < 0 -> 0.
    //    Use prev_filtered = 0 so the LPF output == alpha*raw_deriv,
    //    isolating the gate decision.
    // ---------------------------------------------------------------------
    const double DT = 1.0 / 50.0; // 50 Hz loop -> dt = 0.02

    // Case A: error>0, deriv>0  (error growing further positive) -> KEEP D.
    //   error=+1, prev_error=0 -> raw=+50; product=+50>0 -> pass.
    {
        double out = 0.0;
        double d = dampedDerivative(1.0, 0.0, 0.0, DT, ALPHA, out);
        // hand: raw=50, gate passes, d = 0.2*50 + 0.8*0 = 10
        expect_near(d, 10.0, "gate A: e>0,deriv>0 KEEP D (10)");
        expect_near(out, 10.0, "gate A: out_filtered == d");
    }
    // Case B: error>0, deriv<0  (error shrinking, converging) -> KILL D.
    //   error=+1, prev_error=+2 -> raw=-50; product=-50<0 -> raw:=0.
    {
        double out = 0.0;
        double d = dampedDerivative(1.0, 2.0, 0.0, DT, ALPHA, out);
        // hand: raw gated to 0, d = 0.2*0 + 0.8*0 = 0
        expect_near(d, 0.0, "gate B: e>0,deriv<0 KILL D (0)");
        expect_near(out, 0.0, "gate B: out_filtered == 0");
    }
    // Case C: error<0, deriv<0  (error growing more negative) -> KEEP D.
    //   error=-1, prev_error=0 -> raw=-50; product=+50>0 -> pass.
    {
        double out = 0.0;
        double d = dampedDerivative(-1.0, 0.0, 0.0, DT, ALPHA, out);
        // hand: raw=-50, passes, d = 0.2*(-50) = -10
        expect_near(d, -10.0, "gate C: e<0,deriv<0 KEEP D (-10)");
    }
    // Case D: error<0, deriv>0  (error rising toward zero, converging) -> KILL D.
    //   error=-1, prev_error=-2 -> raw=+50; product=-50<0 -> raw:=0.
    {
        double out = 0.0;
        double d = dampedDerivative(-1.0, -2.0, 0.0, DT, ALPHA, out);
        // hand: raw gated to 0, d = 0
        expect_near(d, 0.0, "gate D: e<0,deriv>0 KILL D (0)");
    }
    // Boundary: product == 0 (error==0) -> NOT < 0 -> raw passes (not gated).
    {
        double out = 0.0;
        double d = dampedDerivative(0.0, -1.0, 0.0, DT, ALPHA, out);
        // raw = (0 - (-1))/0.02 = 50; product = 0*50 = 0; 0 < 0 false -> pass.
        // d = 0.2*50 = 10
        expect_near(d, 10.0, "gate boundary: error==0 passes (not gated)");
    }
    // Boundary: raw == 0 (error == prev_error) -> product 0 -> passes, d uses memory.
    {
        double out = 0.0;
        double d = dampedDerivative(5.0, 5.0, 3.0, DT, ALPHA, out);
        // raw = 0; product = 0 -> not gated; d = 0.2*0 + 0.8*3 = 2.4
        expect_near(d, 2.4, "gate boundary: raw==0 -> LPF carries memory (2.4)");
    }

    // ---------------------------------------------------------------------
    // 5. LPF — 1st-order filter numerics with non-zero memory (gate passes).
    //    out = alpha*raw + (1-alpha)*prev_filtered, alpha=0.2.
    // ---------------------------------------------------------------------
    // error>0,deriv>0 (passes gate), prev_filtered nonzero.
    //   error=+1, prev_error=0 -> raw=+50; prev_filtered=20.
    {
        double out = 0.0;
        double d = dampedDerivative(1.0, 0.0, 20.0, DT, ALPHA, out);
        // hand: d = 0.2*50 + 0.8*20 = 10 + 16 = 26
        expect_near(d, 26.0, "LPF: 0.2*50 + 0.8*20 = 26");
        expect_near(out, 26.0, "LPF: out_filtered mirrors return");
    }
    // Two consecutive ticks with the same raw input converge toward raw.
    {
        double out = 0.0;
        // tick 1: prev_filtered = 0 -> d = 0.2*50 = 10
        double d1 = dampedDerivative(1.0, 0.0, 0.0, DT, ALPHA, out);
        expect_near(d1, 10.0, "LPF tick1: 0.2*50 = 10");
        // tick 2: feed out back as prev_filtered -> d = 0.2*50 + 0.8*10 = 18
        double d2 = dampedDerivative(1.0, 0.0, out, DT, ALPHA, out);
        expect_near(d2, 18.0, "LPF tick2: 0.2*50 + 0.8*10 = 18");
    }
    // Gated derivative still LPFs the memory toward zero.
    //   gate kills raw -> d = 0.2*0 + 0.8*prev = 0.8*prev.
    {
        double out = 0.0;
        double d = dampedDerivative(1.0, 5.0, 50.0, DT, ALPHA, out);
        // raw = (1-5)/0.02 = -200; error*raw = -200 < 0 -> gated to 0.
        // d = 0.2*0 + 0.8*50 = 40
        expect_near(d, 40.0, "LPF: gated raw, memory decays 0.8*50 = 40");
    }

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
