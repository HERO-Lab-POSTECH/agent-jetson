// Characterization test: firmware/agent/dvl_position.cpp
//
// WHAT IS UNDER TEST
// ------------------
// The surviving DVL callback, compiled from THE SHIPPED SOURCE (like
// test_yaw_deadband_ff.cpp does with pid.cpp). Two things are checked:
//
//   1. It still compiles. On 2026-09-04 three callbacks with no publisher were
//      deleted along with the globals only they used; every `extern` this file
//      still needs had to survive that. A missing one is a link error on the
//      board and nothing on a dev machine -- unless this test exists.
//   2. Both branches assign what they used to. `command == 1` clears the
//      integrator state; anything else stores the three targets verbatim.
//
// WHAT IT CANNOT TELL YOU
// -----------------------
// Nothing here reads TARGET_X/Y/Z back. Since the deletion, no code does --
// msgCallback_dvl_velocity held the only reader (`desired_angle_depth =
// temp_depth + TARGET_Z`) and its topic has had no publisher since 2022. See
// the open-circuit note at the top of dvl_position.h. This test pins the
// callback's behaviour, not the usefulness of the values it stores.
//
// Build & run (local, no ROS, no Arduino):
//   c++ -std=c++11 -I. test_firmware_dvl.cpp -o /tmp/t && /tmp/t

#include <cstdio>
#include <cmath>
#include <stdint.h>
#include <Arduino.h>   // host shim: boolean, constrain

// The firmware globals whose DEFINITIONS live in agent.ino, which cannot be
// linked on a host. Types must match agent.ino / the headers exactly -- a
// mismatch is a compile error here and a link error on the board.
volatile double X = 0, Y = 0, Z = 0;
volatile double Tx = 0, Ty = 0;
volatile double error_sum_x = 0, error_sum_y = 0;
int control_T = 0;
volatile double TARGET_X = 0, TARGET_Y = 0, TARGET_Z = 0;
volatile double temp_depth = 0;
volatile double error_x = 0, error_y = 0;
volatile double error_d_x = 0, error_d_y = 0;
volatile double error_x_pre = 0, error_y_pre = 0;
volatile double a_x = 0, a_y = 0;
volatile double error_d_x_pre = 0, error_d_y_pre = 0;
volatile double pre_a_x = 0, pre_a_y = 0;
volatile double pre_v_x = 0, pre_v_y = 0;

// Pulled in by the pid.h -> {thrusters.h, ahrs.h} include chain.
volatile int pwm_m0 = 0, pid_pwm_m0 = 0;
volatile int pwm_m1 = 0, pid_pwm_m1 = 0;
volatile int pwm_m2 = 0, pid_pwm_m2 = 0;
volatile int pwm_m3 = 0, pid_pwm_m3 = 0;
volatile int pwm_m4 = 0, pid_pwm_m4 = 0;
volatile int pwm_m5 = 0, pid_pwm_m5 = 0;
void UART2_write(char) {}
void esc_input(uint8_t, uint16_t, uint16_t, uint16_t) {}
double depth = 0;

#include "../../firmware/agent/dvl_position.cpp"   // THE SHIPPED CODE, not a copy

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
    // command != 1: store the three targets verbatim, touch nothing else.
    {
        X = 7.0; Y = 8.0; error_sum_x = 9.0;
        hero_msgs::hero_agent_dvl m;
        m.command = 0; m.TARGET_X = 1.5f; m.TARGET_Y = -2.5f; m.TARGET_Z = 0.25f;
        msgCallback_dvl(m);
        expect_near(TARGET_X, 1.5, "store: TARGET_X");
        expect_near(TARGET_Y, -2.5, "store: TARGET_Y");
        expect_near(TARGET_Z, 0.25, "store: TARGET_Z");
        expect_near(X, 7.0, "store: X untouched");
        expect_near(Y, 8.0, "store: Y untouched");
        expect_near(error_sum_x, 9.0, "store: error_sum_x untouched");
    }

    // command == 1: full reset, and temp_depth latches the CURRENT depth.
    {
        depth = 3.75;
        X = 7.0; Y = 8.0; Tx = 1.0; Ty = 2.0;
        error_sum_x = 9.0; error_sum_y = 10.0;
        error_x = 1.0; error_d_x = 1.0; a_x = 1.0; pre_v_x = 1.0;
        hero_msgs::hero_agent_dvl m;
        m.command = 1; m.TARGET_X = 99.0f; m.TARGET_Y = 99.0f; m.TARGET_Z = 99.0f;
        msgCallback_dvl(m);
        expect_near(X, 0, "reset: X");
        expect_near(Y, 0, "reset: Y");
        expect_near(Tx, 0, "reset: Tx");
        expect_near(Ty, 0, "reset: Ty");
        expect_near(error_sum_x, 0, "reset: error_sum_x");
        expect_near(error_sum_y, 0, "reset: error_sum_y");
        expect_near(error_x, 0, "reset: error_x");
        expect_near(error_d_x, 0, "reset: error_d_x");
        expect_near(a_x, 0, "reset: a_x");
        expect_near(pre_v_x, 0, "reset: pre_v_x");
        expect_near(temp_depth, 3.75, "reset: temp_depth latches depth");
        expect_near(TARGET_X, 0, "reset: TARGET_X zeroed, msg ignored");
        expect_near(TARGET_Z, 0, "reset: TARGET_Z zeroed, msg ignored");
    }

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
