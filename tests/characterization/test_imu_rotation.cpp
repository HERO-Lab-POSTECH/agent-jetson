// Characterization test: imuCallback IMU yaw-offset rotation
//
// Pins the CURRENT behavior of albc_controller.cpp:205-214 (imuCallback) — how
// raw IMU RPY is mapped into the robot body frame:
//   raw_roll  =  ROLL
//   raw_pitch = -(PITCH)                       <-- PITCH is NEGATED
//   c = cos(offset_rad), s = sin(offset_rad)
//   out_roll  =  c*raw_roll + s*raw_pitch
//   out_pitch = -s*raw_roll + c*raw_pitch
//   out_yaw   =  YAW                            <-- pass-through
//
// Two layers of checking:
//   (1) self-pin   — call rotateImu() and EXPECT its output ("current = truth").
//   (2) independent — at clean angles (0, 90 deg) recompute the closed form by
//       hand so an oracle typo (wrong sign / swapped term) is also caught.
//
// Build & run (local, no ROS):
//   c++ -std=c++11 -Wall \
//     -I/Users/kimseungmin/ksm_Obsidian/0_Project/in_progress/albc/code/agent-jetson/tests/characterization \
//     -I/Users/kimseungmin/ksm_Obsidian/0_Project/in_progress/albc/code/agent-jetson/robot/albc_control/include \
//     test_imu_rotation.cpp -o /tmp/t && /tmp/t

#include "albc_control/imu_rotation.h"
#include "albc_control/albc_kinematics.h"   // DEG2RAD
#include <cstdio>
#include <cmath>

static int failures = 0, checks = 0;

static void expect_near(double got, double want, const char* desc)
{
    checks++;
    if (!std::isfinite(got) || std::fabs(got - want) > 1e-9) {
        failures++;
        std::printf("FAIL [%s]: got %.12f want %.12f\n", desc, got, want);
    }
}

int main()
{
    // -----------------------------------------------------------------
    // (A) Self-pin at the default 45 deg offset.
    //     We EXPECT the oracle's own output: this freezes "current = truth".
    //     We additionally recompute the SAME closed form independently below
    //     so an oracle transcription typo would surface as a (B)/(C) failure.
    // -----------------------------------------------------------------
    const double off45 = DEG2RAD(45.0);
    const double c45 = std::cos(off45), s45 = std::sin(off45);

    {
        // arbitrary, non-symmetric input so every term is exercised
        const double R = 3.0, P = 7.0, Y = 1.5;
        ImuRpy o = rotateImu(R, P, Y, off45);
        double raw_roll = R, raw_pitch = -(P);
        // self-pin: independent recomputation of the exact same formula
        expect_near(o.roll,   c45 * raw_roll + s45 * raw_pitch, "45deg: roll = c*rr + s*rp");
        expect_near(o.pitch, -s45 * raw_roll + c45 * raw_pitch, "45deg: pitch = -s*rr + c*rp");
        expect_near(o.yaw,    Y,                                "45deg: yaw pass-through");
    }

    {
        // negative inputs too
        const double R = -2.0, P = -4.0, Y = -0.25;
        ImuRpy o = rotateImu(R, P, Y, off45);
        double raw_roll = R, raw_pitch = -(P);
        expect_near(o.roll,   c45 * raw_roll + s45 * raw_pitch, "45deg neg: roll");
        expect_near(o.pitch, -s45 * raw_roll + c45 * raw_pitch, "45deg neg: pitch");
        expect_near(o.yaw,    Y,                                "45deg neg: yaw");
    }

    // -----------------------------------------------------------------
    // (B) offset = 0  => identity rotation. c=1, s=0.
    //     out_roll  = raw_roll  =  ROLL
    //     out_pitch = raw_pitch = -(PITCH)      <-- proves the PITCH negation
    //     out_yaw   = YAW
    //     Hand values, NOT taken from the oracle.
    // -----------------------------------------------------------------
    {
        const double R = 11.0, P = 5.0, Y = 9.0;
        ImuRpy o = rotateImu(R, P, Y, 0.0);
        expect_near(o.roll,   11.0,  "off0: roll = ROLL");
        expect_near(o.pitch, -5.0,   "off0: pitch = -PITCH (negation pinned)");
        expect_near(o.yaw,    9.0,   "off0: yaw = YAW");
    }

    // -----------------------------------------------------------------
    // (C) offset = 90 deg => c=0, s=1. Pure axis swap, hand-computed.
    //     out_roll  =  0*raw_roll + 1*raw_pitch =  raw_pitch = -(PITCH)
    //     out_pitch = -1*raw_roll + 0*raw_pitch = -raw_roll  = -ROLL
    //     out_yaw   =  YAW
    // -----------------------------------------------------------------
    {
        const double R = 2.0, P = 6.0, Y = 0.5;
        ImuRpy o = rotateImu(R, P, Y, DEG2RAD(90.0));
        // raw_pitch = -(6) = -6  => out_roll = -6 ; out_pitch = -ROLL = -2
        expect_near(o.roll,  -6.0,  "off90: roll = raw_pitch = -PITCH");
        expect_near(o.pitch, -2.0,  "off90: pitch = -raw_roll = -ROLL");
        expect_near(o.yaw,    0.5,  "off90: yaw = YAW");
    }

    // -----------------------------------------------------------------
    // (D) offset = 180 deg => c=-1, s=0.  Hand-computed.
    //     out_roll  = -raw_roll  = -ROLL
    //     out_pitch = -raw_pitch =  PITCH
    // -----------------------------------------------------------------
    {
        const double R = 4.0, P = 3.0, Y = -1.0;
        ImuRpy o = rotateImu(R, P, Y, DEG2RAD(180.0));
        expect_near(o.roll,  -4.0,  "off180: roll = -ROLL");
        expect_near(o.pitch,  3.0,  "off180: pitch = +PITCH");
        expect_near(o.yaw,   -1.0,  "off180: yaw = YAW");
    }

    // -----------------------------------------------------------------
    // (E) Zero input => zero out_roll/out_pitch at any offset; yaw still passes.
    // -----------------------------------------------------------------
    {
        ImuRpy o = rotateImu(0.0, 0.0, 1.23, off45);
        expect_near(o.roll,  0.0,  "zero: roll = 0");
        expect_near(o.pitch, 0.0,  "zero: pitch = 0");
        expect_near(o.yaw,   1.23, "zero: yaw passes through");
    }

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
