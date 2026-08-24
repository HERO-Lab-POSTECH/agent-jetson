// Characterization test: classic yaw ESC-deadband feedforward (2026-08-24)
//
// WHAT IS UNDER TEST
// ------------------
// firmware/agent/pid.cpp PID_control_yaw(), after the deadband feedforward and
// the 3-channel (m4-excluded) reallocation. This file COMPILES THE SHIPPED
// pid.cpp -- it is not an oracle copy -- so the arithmetic checked here is the
// arithmetic the board runs. That is why tests/characterization/Arduino.h
// exists; see its header for why the shim lives on this side of the tree.
//
// WHY IT EXISTS
// -------------
// The 2026-08-24 tank bag showed the yaw loop NEVER left the ESC deadband:
// 0 escapes in 16,340 samples over 830 s, max |PID_yaw| = 44.22 against a
// threshold of 45. The loop was structurally open. The fix inverts the deadband
// exactly the way the RL mixer already does (thruster_mixer.undeadband,
// D = 0.15), so the two control paths finally agree on the plant inverse.
//
// Analysis + every number quoted below:
//   0_Project/in_progress/albc/notes/2026-08-24-fault-tolerant-allocation-analysis.md
//   sections 3-2 (measurement), 3-5 (replay table), 3-6 (recommendation).
//
// It also pins the two agent.ino edits that MUST ship in the same flash as the
// feedforward, by reading the source text -- there is no way to link agent.ino
// on a host. The boot-setpoint one is the important half: with the deadband
// swallowing everything, `desired_angle_yaw = 1` (57.3 deg) was harmless; with
// the feedforward in, power-up plus 'Y' slews the robot 57 degrees.
//
// Build & run (local, no ROS):
//   tests/characterization/run.sh
// or by hand from this directory:
//   c++ -std=c++11 -Wall -I. test_yaw_deadband_ff.cpp -o /tmp/t && /tmp/t

#include <cstdio>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

// ---------------------------------------------------------------------------
// Firmware globals. pid.h only `extern`s these -- agent.ino owns the real
// definitions, and agent.ino cannot be linked on a host (ROS, Servo, MS5837).
// Values match agent.ino:126-149 so the gains under test are the shipped gains.
// ---------------------------------------------------------------------------
volatile int pwm_m0, pwm_m1, pwm_m2, pwm_m3, pwm_m4, pwm_m5;
volatile float yaw, acc_yaw;

volatile uint8_t cont_yaw_on = 0;
volatile uint8_t cont_depth_on = 0;
volatile double T = 0.004;        // agent.ino:126
volatile double T_depth = 0.04;   // agent.ino:127
volatile int throttle = 40;       // agent.ino:129 -- unused by the 3-channel mixing
volatile double P_angle_gain_yaw = 6.5;  // agent.ino:131
volatile double P_gain_yaw = 10;         // agent.ino:132
volatile double I_gain_yaw = 1;          // agent.ino:133
volatile double D_gain_yaw = 0.5;        // agent.ino:134
volatile double error_yaw;
volatile double error_pid_yaw, error_pid_yaw1;
volatile double P_angle_pid_yaw;
volatile double P_yaw, I_yaw, D_yaw, PID_yaw;
volatile double desired_angle_yaw = 0;   // agent.ino:144 (was 1 before 2026-08-24)

double depth = 0.0;
double P_gain_depth = 1100.0, I_gain_depth = 10.0, D_gain_depth = 100.0;
double error_pid_depth, error_pid_depth1;
double P_angle_pid_depth;
double P_depth, I_depth, D_depth, PID_depth;
double desired_angle_depth = 0.5;

volatile uint8_t cont_direc = 0;
volatile int move_speed = 20;
volatile int Th_0, Th_1, Th_2, Th_3;
volatile int darknet_Th_0, darknet_Th_1, darknet_Th_2, darknet_Th_3;

#include "../../firmware/agent/pid.cpp"   // THE SHIPPED CODE, not a copy

// ---------------------------------------------------------------------------
#ifndef AGENT_SRC_DIR
#define AGENT_SRC_DIR "../../firmware/agent"
#endif

static int failures = 0, checks = 0;

static void expect_near(double got, double want, double tol, const char *desc)
{
    checks++;
    if (std::fabs(got - want) > tol) {
        failures++;
        std::printf("FAIL [%s]: got %.6f want %.6f (tol %g)\n", desc, got, want, tol);
    }
}

static void expect_true(bool cond, const char *desc)
{
    checks++;
    if (!cond) {
        failures++;
        std::printf("FAIL [%s]\n", desc);
    }
}

// Drive PID_control_yaw() so that PID_yaw reaches `target` BEFORE the
// feedforward, then return the post-feedforward value.
//
//   PID_yaw = P_yaw + D_yaw + I_yaw
//           = 10*e + 0 + 0.004*e        (D zeroed by seeding error_pid_yaw1 = e,
//                                        I zeroed on entry so it accrues one step)
//   e = P_angle_gain_yaw * (desired - yaw) - acc_yaw
static double drive(double target, int direc, int speed)
{
    const double e = target / (P_gain_yaw + T * I_gain_yaw);
    I_yaw = 0.0;
    error_pid_yaw1 = e;
    acc_yaw = 0.0f;
    desired_angle_yaw = 0.0;
    yaw = (float)(-e / P_angle_gain_yaw);
    cont_direc = (uint8_t)direc;
    move_speed = speed;
    PID_control_yaw();
    return PID_yaw;
}

// The PWM offset the yaw loop actually put on a channel.
static int off(volatile int &pwm) { return (int)pwm - ESC_NEUTRAL; }

static std::string read_agent_ino()
{
    std::ifstream f((std::string(AGENT_SRC_DIR) + "/agent.ino").c_str());
    if (!f) return std::string();
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// One `else if (Command == 'X')` block, from its anchor to the start of the next
// branch. Bounded by the real block boundary rather than a byte count: the
// comments inside these blocks are Korean (3 bytes per char), so a fixed window
// silently shrinks the moment someone documents an edit.
static std::string key_block(const std::string &hay, const std::string &anchor)
{
    size_t p = hay.find(anchor);
    if (p == std::string::npos) return std::string();
    size_t q = hay.find("Command == '", p + anchor.size());
    return hay.substr(p, q == std::string::npos ? std::string::npos : q - p);
}

int main()
{
    const double SPAN = ESC_MAX - ESC_NEUTRAL;   // 300

    // -----------------------------------------------------------------
    // (A) Below EPS the loop commands EXACTLY neutral.
    //     Without this the feedforward would lift every speck of numerical
    //     noise straight to 45 counts and the robot would buzz at rest.
    // -----------------------------------------------------------------
    {
        expect_near(YAW_FF_EPS, 4.0, 1e-12, "EPS is the chosen 4.0 counts");
        expect_true(YAW_FF_EPS >= 3.0 && YAW_FF_EPS <= 6.0,
                    "EPS stays inside the 3-6 band the replay justified");
        const double vals[] = {0.0, 0.5, -0.5, 3.9, -3.9};
        for (int i = 0; i < 5; ++i) {
            double post = drive(vals[i], 0, 0);
            expect_near(post, 0.0, 1e-12, "below EPS -> PID_yaw forced to 0");
            expect_near(off(pwm_m2), 0.0, 1e-9, "below EPS -> m2 exactly neutral");
            expect_near(off(pwm_m5), 0.0, 1e-9, "below EPS -> m5 exactly neutral");
        }
    }

    // -----------------------------------------------------------------
    // (B) Just above EPS the command CLEARS the deadband in one step.
    //     This is the whole point: the old loop asymptotically approached
    //     45 from below and never crossed it.
    // -----------------------------------------------------------------
    {
        double post = drive(4.1, 0, 0);
        expect_true(std::fabs(post) >= YAW_FF_DEADBAND,
                    "just above EPS -> command already outside the ESC deadband");
        expect_near(post, YAW_FF_DEADBAND + YAW_FF_SLOPE * 4.1, 1e-3,
                    "just above EPS -> exact feedforward value");
        // and it reaches the ESC as a real pulse offset, sign-inverted per the
        // channel convention (pwm = -PID_yaw + NEUTRAL).
        expect_near(off(pwm_m2), -post, 1.0, "m2 = -PID_yaw + NEUTRAL");
        expect_near(off(pwm_m5), -post, 1.0, "m5 = -PID_yaw + NEUTRAL");
    }

    // -----------------------------------------------------------------
    // (C) Sign is preserved and the map is odd.
    // -----------------------------------------------------------------
    {
        double pos = drive(20.0, 0, 0);
        double neg = drive(-20.0, 0, 0);
        expect_near(pos, -neg, 1e-4, "feedforward is an odd function");
        expect_true(pos > 0.0, "positive stays positive");
        expect_true(neg < 0.0, "negative stays negative");
    }

    // -----------------------------------------------------------------
    // (D) Full scale maps to full scale -- no authority gained, none lost.
    //     |u| = span must land on exactly span, or the ESC clips and the
    //     top of the range becomes bang-bang.
    // -----------------------------------------------------------------
    {
        double post = drive(SPAN, 0, 0);
        expect_near(post, SPAN, 1e-2, "|u| = span -> exactly span (no clipping)");
        expect_near(YAW_FF_SLOPE, 1.0 - YAW_FF_DEADBAND / SPAN, 1e-12,
                    "SLOPE is derived from DEADBAND, not an independent literal");
    }

    // -----------------------------------------------------------------
    // (E) HOMOMORPHISM with the RL mixer. Normalised by span, the firmware
    //     feedforward must equal thruster_mixer.undeadband(a, 0.15) exactly.
    //     If these ever drift apart, the two control paths are inverting
    //     different plants and the tank cannot compare them.
    // -----------------------------------------------------------------
    {
        const double D = YAW_FF_DEADBAND / SPAN;    // 0.15
        expect_near(D, 0.15, 1e-12, "normalised deadband equals the mixer's 0.15");
        const double as[] = {0.05, 0.2, 0.5, 1.0};
        for (int i = 0; i < 4; ++i) {
            double a = as[i];
            double mixer = D + (1.0 - D) * a;            // undeadband(), positive branch
            double fw = drive(a * SPAN, 0, 0) / SPAN;    // firmware, normalised
            expect_near(fw, mixer, 1e-4, "firmware FF == RL mixer undeadband");
        }
    }

    // -----------------------------------------------------------------
    // (F) The measured peak of the dead 830 s segment. |PID_yaw| = 44.22 was
    //     the largest command the old loop ever produced and it produced NO
    //     thrust. Post-fix it becomes 82.59 counts -- the replay table's 82.6
    //     max, and comfortably below span (that is the difference from the
    //     rejected "P x14" option, which peaked at 613 counts).
    // -----------------------------------------------------------------
    {
        double post = drive(44.22, 0, 0);
        expect_near(post, 82.587, 0.01, "measured peak 44.22 -> 82.59 counts");
        expect_true(post < SPAN, "measured peak stays far from saturation");
        // the same input under the OLD code produced 44.22, i.e. below 45 -> nothing
        expect_true(44.22 < YAW_FF_DEADBAND, "44.22 really was inside the deadband");
    }

    // -----------------------------------------------------------------
    // (G) Integrator clamp is +-30, not +-100.
    //     With the feedforward in, the integrator touches the output for the
    //     first time; +-100 is a third of span and would sit there as bias.
    // -----------------------------------------------------------------
    {
        expect_near(YAW_I_CLAMP, 30.0, 1e-12, "I clamp lowered to 30");
        I_yaw = 0.0;
        error_pid_yaw1 = 0.0;
        acc_yaw = 0.0f;
        desired_angle_yaw = 1.0;   // large standing error
        yaw = 0.0f;
        cont_direc = 0;
        move_speed = 0;
        for (int i = 0; i < 100000; ++i) PID_control_yaw();
        expect_near(I_yaw, YAW_I_CLAMP, 1e-9, "I_yaw saturates at +YAW_I_CLAMP");
        desired_angle_yaw = -1.0;
        for (int i = 0; i < 100000; ++i) PID_control_yaw();
        expect_near(I_yaw, -YAW_I_CLAMP, 1e-9, "I_yaw saturates at -YAW_I_CLAMP");
    }

    // -----------------------------------------------------------------
    // (H) m4 stays at exact neutral in EVERY branch, and m1 never carries a
    //     yaw term. This is the 3-channel reallocation (commit 3771674); the
    //     feedforward must not have reintroduced either.
    // -----------------------------------------------------------------
    {
        for (int d = 0; d <= 4; ++d) {
            drive(30.0, d, 50);
            expect_near(off(pwm_m4), 0.0, 1e-9, "m4 excluded -> exact neutral");
            // m1 = NEUTRAL -/+ move_speed only. No PID_yaw term in any branch.
            int want_m1 = (d == 1 || d == 3) ? -50 : (d == 2 || d == 4) ? 50 : 0;
            expect_near(off(pwm_m1), want_m1, 1e-9, "m1 carries translation only");
        }
        // throttle must be dead: it is a nullspace term that no longer has a
        // nullspace to live in once m4 is gone (see pid.cpp comment).
        throttle = 999;
        drive(0.0, 0, 0);
        expect_near(off(pwm_m1), 0.0, 1e-9, "throttle no longer reaches m1");
        expect_near(off(pwm_m2), 0.0, 1e-9, "throttle no longer reaches m2");
        expect_near(off(pwm_m5), 0.0, 1e-9, "throttle no longer reaches m5");
        throttle = 40;
    }

    // -----------------------------------------------------------------
    // (I) Source guards on agent.ino. These cannot be linked, so they are
    //     checked as text. A missing edit here means the feedforward ships
    //     with a live 57-degree boot setpoint.
    // -----------------------------------------------------------------
    {
        std::string ino = read_agent_ino();
        expect_true(!ino.empty(), "agent.ino is readable (AGENT_SRC_DIR correct)");
        if (!ino.empty()) {
            expect_true(ino.find("volatile double desired_angle_yaw = 0;") != std::string::npos,
                        "boot setpoint is 0 rad");
            expect_true(ino.find("volatile double desired_angle_yaw = 1;") == std::string::npos,
                        "the old 1 rad (57.3 deg) boot setpoint is gone");
            std::string z = key_block(ino, "Command == 'Z'");
            expect_true(!z.empty() && z.find("I_yaw = 0;") != std::string::npos,
                        "'Z' (yaw reset) clears the integrator");
            std::string y = key_block(ino, "Command == 'Y'");
            expect_true(!y.empty() && y.find("if (cont_yaw_on) I_yaw = 0;") != std::string::npos,
                        "'Y' clears the integrator on the rising edge only");
            // rising edge ONLY: an unconditional reset would also fire when the
            // operator switches yaw control OFF, which is harmless today but
            // hides the intent. Guard the shape, not just the presence.
            int n = 0;
            for (size_t p = y.find("I_yaw = 0;"); p != std::string::npos;
                 p = y.find("I_yaw = 0;", p + 1)) ++n;
            expect_true(n == 1, "'Y' has no second, unconditional I_yaw reset");
        }
    }

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
