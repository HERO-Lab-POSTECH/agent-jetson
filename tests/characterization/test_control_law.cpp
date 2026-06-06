// Characterization test: albc_controller computeControlOutput (4 modes)
//
// Pins the CURRENT behavior of the Control Law switch (albc_controller.cpp:385-433)
// as a ROS-free pure function. The redesign must keep (target_x, target_y)
// byte-identical for the same inputs.
//
// Strategy (chain1 Phase 1):
//   - SELF-PIN: every EXPECT compares against computeControlOutputOracle(),
//     the byte-identical transcription. "current implementation = ground truth".
//   - INDEPENDENT CROSS-CHECK: for clean inputs the math is also recomputed by
//     hand here (separate arithmetic), so an oracle transcription typo is caught
//     even though the oracle is the pin.
//
// Modes (1=TDC, 2=PID, 3=FIXED, 4=MANUAL):
//   TDC   — equilibrium hold inside the level band (dy/dx forced to 0), and
//           normal incremental-P outside the band.
//   PID   — target recomputed only outside the level band; held otherwise.
//   FIXED — one exponential step toward (-L2, +L1).
//   MANUAL— no-op (target unchanged).
//
// Build & run (local, no ROS):
//   c++ -std=c++11 -Wall -I. -I../../robot/albc_control/include \
//       test_control_law.cpp -o /tmp/t && /tmp/t

#include "albc_control/control_law.h"
#include <cstdio>
#include <cmath>

static int failures = 0, checks = 0;

static void expect_near(double got, double want, const char* desc) {
    checks++;
    if (std::fabs(got - want) > 1e-9) {
        failures++;
        std::printf("FAIL [%s]: got %.12f want %.12f\n", desc, got, want);
    }
}

int main() {
    // --- Representative gains (arbitrary but fixed; values irrelevant to the pin) ---
    const double M_TD = 2.0, KP_TD = 0.5;
    const double KP_R = 1.0, KI_R = 0.1, KD_R = 0.2;
    const double KP_P = 1.5, KI_P = 0.3, KD_P = 0.4;

    auto base = [&](int mode) {
        CtrlIn in = {};
        in.mode = mode;
        in.M_td = M_TD; in.Kp_td = KP_TD;
        in.kp_roll = KP_R; in.ki_roll = KI_R; in.kd_roll = KD_R;
        in.kp_pitch = KP_P; in.ki_pitch = KI_P; in.kd_pitch = KD_P;
        return in;
    };

    // ============================================================
    // TDC (mode 1)
    // ============================================================

    // (1a) TDC equilibrium hold — both axes INSIDE level band: dy=dx=0, target unchanged.
    {
        CtrlIn in = base(CTRL_TDC);
        in.current_roll = 0.001;  in.current_pitch = -0.001;   // |.| < LEVEL_THRESHOLD (0.01745)
        in.error_roll   = 0.5;    in.error_pitch   = -0.3;     // nonzero, but must be clamped out
        in.target_x = 0.10; in.target_y = -0.20;
        CtrlOut o = computeControlOutputOracle(in);
        expect_near(o.target_x, in.target_x, "TDC hold(in-band): target_x frozen");
        expect_near(o.target_y, in.target_y, "TDC hold(in-band): target_y frozen");
    }

    // (1b) TDC — roll OUTSIDE band, pitch INSIDE band: only target_y moves.
    {
        CtrlIn in = base(CTRL_TDC);
        in.current_roll = 0.10;   in.current_pitch = 0.001;    // roll out, pitch in
        in.error_roll   = 0.2;    in.error_pitch   = 0.9;      // pitch error must be clamped (in-band)
        in.target_x = 0.0; in.target_y = 0.0;
        CtrlOut o = computeControlOutputOracle(in);

        // Independent recompute of the TDC math for the y-axis:
        double denom = std::fabs(Fb * std::cos(in.current_roll) * std::cos(in.current_pitch));
        double cf = std::min(Fb / std::max(denom, COS_EPSILON), COMMON_FACTOR_MAX);
        double dy_hand = cf * (M_TD * KP_TD * in.error_roll);   // roll out-of-band -> applied
        expect_near(o.target_y, in.target_y + dy_hand, "TDC: target_y += cf*M*Kp*err_roll (hand)");
        expect_near(o.target_x, in.target_x, "TDC: target_x frozen (pitch in-band)");
    }

    // (1c) TDC normal — both axes OUTSIDE band: dy and dx both applied, signs pinned.
    {
        CtrlIn in = base(CTRL_TDC);
        in.current_roll = 0.05;   in.current_pitch = 0.05;     // both out-of-band
        in.error_roll   = 0.4;    in.error_pitch   = 0.25;
        in.target_x = 1.0; in.target_y = -1.0;
        CtrlOut o = computeControlOutputOracle(in);

        double denom = std::fabs(Fb * std::cos(in.current_roll) * std::cos(in.current_pitch));
        double cf = std::min(Fb / std::max(denom, COS_EPSILON), COMMON_FACTOR_MAX);
        double dy_hand =  cf * (M_TD * KP_TD * in.error_roll);
        double dx_hand = -cf * (M_TD * KP_TD * in.error_pitch);   // NOTE the minus sign (pinned)
        expect_near(o.target_y, in.target_y + dy_hand, "TDC normal: target_y (hand)");
        expect_near(o.target_x, in.target_x + dx_hand, "TDC normal: target_x = -cf*... (sign pinned)");
    }

    // (1d) TDC common_factor saturation — clamps at COMMON_FACTOR_MAX (=10).
    // At roll=pitch=0 denom=|Fb|, Fb/denom = 1 < 10, so to force saturation we use
    // a large tilt that makes cos small -> Fb/denom large -> clamp to 10.
    {
        CtrlIn in = base(CTRL_TDC);
        in.current_roll = 1.50;   in.current_pitch = 1.50;     // cos(1.5) tiny -> ratio >> 10
        in.error_roll   = 0.1;    in.error_pitch   = 0.0;
        in.target_x = 0.0; in.target_y = 0.0;
        CtrlOut o = computeControlOutputOracle(in);

        double denom = std::fabs(Fb * std::cos(in.current_roll) * std::cos(in.current_pitch));
        double ratio = Fb / std::max(denom, COS_EPSILON);
        // sanity: this input really does saturate
        if (!(ratio > COMMON_FACTOR_MAX)) {
            checks++; failures++;
            std::printf("FAIL [TDC sat setup]: ratio %.6f not > %.1f\n", ratio, COMMON_FACTOR_MAX);
        }
        double cf = std::min(ratio, COMMON_FACTOR_MAX);   // == 10
        double dy_hand = cf * (M_TD * KP_TD * in.error_roll);
        expect_near(cf, COMMON_FACTOR_MAX, "TDC: common_factor saturates at 10");
        expect_near(o.target_y, dy_hand, "TDC saturated: target_y uses clamped cf");
    }

    // ============================================================
    // PID (mode 2)
    // ============================================================

    // (2a) PID level band — both axes INSIDE band: target held (no recompute).
    {
        CtrlIn in = base(CTRL_PID);
        in.current_roll = 0.005;  in.current_pitch = -0.005;   // both in-band
        in.error_roll = 0.3; in.error_pitch = 0.3;
        in.integral_roll = 1.0; in.integral_pitch = 1.0;
        in.deriv_roll = 0.5; in.deriv_pitch = 0.5;
        in.target_x = 0.123; in.target_y = -0.456;
        CtrlOut o = computeControlOutputOracle(in);
        expect_near(o.target_x, in.target_x, "PID hold(in-band): target_x held");
        expect_near(o.target_y, in.target_y, "PID hold(in-band): target_y held");
    }

    // (2b) PID active — both axes OUTSIDE band: full PID, base offsets + pitch sign pinned.
    {
        CtrlIn in = base(CTRL_PID);
        in.current_roll = 0.10;   in.current_pitch = -0.10;    // both out-of-band
        in.error_roll = 0.20; in.error_pitch = -0.15;
        in.integral_roll = 2.0; in.integral_pitch = -1.0;
        in.deriv_roll = 0.05; in.deriv_pitch = 0.07;
        in.target_x = 99.0; in.target_y = 99.0;   // must be overwritten (not added to)
        CtrlOut o = computeControlOutputOracle(in);

        double ty_hand = PID_BASE_Y + KP_R*in.error_roll + KI_R*in.integral_roll + KD_R*in.deriv_roll;
        double tx_hand = PID_BASE_X + (-1.0)*(KP_P*in.error_pitch + KI_P*in.integral_pitch + KD_P*in.deriv_pitch);
        expect_near(o.target_y, ty_hand, "PID active: target_y = PID_BASE_Y + roll PID (hand)");
        expect_near(o.target_x, tx_hand, "PID active: target_x = PID_BASE_X - pitch PID (sign pinned)");
        // base offsets are L1 / -L2
        expect_near(PID_BASE_Y, L1,  "PID_BASE_Y == +L1");
        expect_near(PID_BASE_X, -L2, "PID_BASE_X == -L2");
    }

    // (2c) PID mixed band — roll out, pitch in: y recomputed, x held.
    {
        CtrlIn in = base(CTRL_PID);
        in.current_roll = 0.10;   in.current_pitch = 0.001;    // roll out, pitch in
        in.error_roll = 0.20; in.error_pitch = 5.0;            // pitch err ignored (held)
        in.integral_roll = 0.0; in.integral_pitch = 0.0;
        in.deriv_roll = 0.0; in.deriv_pitch = 0.0;
        in.target_x = 0.777; in.target_y = 0.0;
        CtrlOut o = computeControlOutputOracle(in);

        double ty_hand = PID_BASE_Y + KP_R*in.error_roll + KI_R*in.integral_roll + KD_R*in.deriv_roll;
        expect_near(o.target_y, ty_hand, "PID mixed: target_y recomputed (roll out-of-band)");
        expect_near(o.target_x, in.target_x, "PID mixed: target_x held (pitch in-band)");
    }

    // ============================================================
    // FIXED (mode 3) — one exponential step toward (-L2, +L1), alpha=0.05.
    // ============================================================
    {
        CtrlIn in = base(CTRL_FIXED);
        in.target_x = 0.0; in.target_y = 0.0;   // start away from goal
        CtrlOut o = computeControlOutputOracle(in);

        const double ALPHA = 0.05;
        double tx_hand = in.target_x + ALPHA * (-L2 - in.target_x);
        double ty_hand = in.target_y + ALPHA * ( L1 - in.target_y);
        expect_near(o.target_x, tx_hand, "FIXED: target_x += a*(-L2 - target_x) (hand)");
        expect_near(o.target_y, ty_hand, "FIXED: target_y += a*( L1 - target_y) (hand)");
        // direction sanity: one step must move toward the goal
        if (!(std::fabs(o.target_x - (-L2)) < std::fabs(in.target_x - (-L2)))) {
            checks++; failures++;
            std::printf("FAIL [FIXED converge x]: did not move toward -L2\n");
        }
        if (!(std::fabs(o.target_y - ( L1)) < std::fabs(in.target_y - ( L1)))) {
            checks++; failures++;
            std::printf("FAIL [FIXED converge y]: did not move toward L1\n");
        }
    }

    // (3b) FIXED at goal — already at (-L2, +L1): step is zero (fixed point).
    {
        CtrlIn in = base(CTRL_FIXED);
        in.target_x = -L2; in.target_y = L1;
        CtrlOut o = computeControlOutputOracle(in);
        expect_near(o.target_x, -L2, "FIXED fixed-point: target_x stays at -L2");
        expect_near(o.target_y,  L1, "FIXED fixed-point: target_y stays at  L1");
    }

    // ============================================================
    // MANUAL (mode 4) — no-op: target unchanged regardless of all inputs.
    // ============================================================
    {
        CtrlIn in = base(CTRL_MANUAL);
        in.current_roll = 0.9; in.current_pitch = 0.9;
        in.error_roll = 5.0; in.error_pitch = -5.0;
        in.integral_roll = 9.0; in.integral_pitch = 9.0;
        in.deriv_roll = 9.0; in.deriv_pitch = 9.0;
        in.target_x = 0.314; in.target_y = -0.271;
        CtrlOut o = computeControlOutputOracle(in);
        expect_near(o.target_x, in.target_x, "MANUAL: target_x unchanged");
        expect_near(o.target_y, in.target_y, "MANUAL: target_y unchanged");
    }

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
