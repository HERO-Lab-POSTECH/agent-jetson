// Production header: Dashboard — ANSI terminal status readout.
//
// Absorbs the former global printDashboard() (albc_controller.cpp:281-315):
// the clear-and-redraw of mode, roll/pitch, target/FK, joints, gains, motor
// current, and the key legend. The ANSI escapes, box-drawing characters, and
// every printf format/wording are preserved byte-for-byte — only the data
// source changes (file-scope globals -> passed-in references + scalars).
//
// Display data is supplied by the caller:
//   - mode/submode  : from ModeManager
//   - attitude/target/gains : from AttitudeController getters
//   - theta1/2, current_x/y, target_length, current_roll/pitch : scalars
//   - joint currents : scalars
//
// ROS-free: depends only on <cstdio> + the pure helper headers (albc_kinematics
// for RAD2DEG/SAFE_ARM_LENGTH, mode_manager + attitude_controller for the
// readout sources). No ros/ros.h, no std_msgs.

#ifndef ALBC_CONTROL_DASHBOARD_H
#define ALBC_CONTROL_DASHBOARD_H

#include <cstdio>
#include "albc_control/albc_kinematics.h"        // RAD2DEG, SAFE_ARM_LENGTH
#include "albc_control/mode_manager.h"            // ControlMode, ManualSubMode, controlModeName, ModeManager
#include "albc_control/attitude_controller.h"     // AttitudeController getters

namespace albc {

class Dashboard {
public:
    // Render one frame. Byte-identical to the former printDashboard()
    // (albc_controller.cpp:281-315): same ANSI clear, same box-drawing,
    // same printf format strings and key legend.
    static void render(const ModeManager& mode_mgr,
                       const AttitudeController& attitude,
                       double theta1, double theta2,
                       double current_x, double current_y,
                       double target_length,
                       double current_roll, double current_pitch,
                       float joint_current1_mA, float joint_current2_mA) {
        printf("\033[2J\033[H");
        printf("═══════════════════════════════════════════════════\n");
        if (mode_mgr.mode() == ControlMode::MANUAL) {
            const char* sub = (mode_mgr.manualSubmode() == ManualSubMode::JOINT) ? "JOINT" : "POSITION";
            printf("          ALBC Controller [MANUAL:%s]\n", sub);
        } else {
            printf("            ALBC Controller [%s]\n", controlModeName(mode_mgr.mode()));
        }
        printf("═══════════════════════════════════════════════════\n");
        printf(" Roll  %+7.2f / %+7.2f deg  (err %+.2f)\n",
               RAD2DEG(current_roll), RAD2DEG(attitude.targetRoll()), RAD2DEG(attitude.errorRoll()));
        printf(" Pitch %+7.2f / %+7.2f deg  (err %+.2f)\n",
               RAD2DEG(current_pitch), RAD2DEG(attitude.targetPitch()), RAD2DEG(attitude.errorPitch()));
        printf("───────────────────────────────────────────────────\n");
        printf(" Target (%+.4f, %+.4f)   FK (%+.4f, %+.4f)\n",
               attitude.targetX(), attitude.targetY(), current_x, current_y);
        printf(" Joints J1=%.1f  J2=%.1f deg   Len=%.4f/%.4f\n",
               RAD2DEG(theta1), RAD2DEG(theta2), target_length, SAFE_ARM_LENGTH);
        printf("───────────────────────────────────────────────────\n");
        printf(" Gains  mult=%.2f  M=%.4f  Kp=%.3f  Kd=%.1f\n",
               attitude.gainMult(), attitude.Mtd(), attitude.Kptd(), attitude.Kdtd());
        printf(" Motor  J1=%+.0f mA  J2=%+.0f mA\n",
               joint_current1_mA, joint_current2_mA);
        printf("───────────────────────────────────────────────────\n");
        if (mode_mgr.mode() == ControlMode::MANUAL)
            printf(" Keys  ==Cycle  1-4=Mode  w/s/a/d=Adjust  m=SubMode\n");
        else
            printf(" Keys  ==Cycle  1=TDC  2=PID  3=FIXED  4=MANUAL\n");
        printf("═══════════════════════════════════════════════════\n");
        fflush(stdout);
    }
};

}  // namespace albc

#endif  // ALBC_CONTROL_DASHBOARD_H
