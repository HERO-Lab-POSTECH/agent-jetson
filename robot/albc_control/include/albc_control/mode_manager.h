// Production header: ModeManager — control-mode FSM + terminal key handling.
//
// Absorbs the former global control-mode state and key handling from
// albc_controller.cpp:
//   - enum ControlMode / ManualSubMode + controlModeName()  (:52-68)
//   - globals control_mode, manual_submode, manual_theta1/2_deg, manual_x/y
//     (:92, :100-102)
//   - raw-terminal keyboard (initKeyboard/closeKeyboard/readKey)  (:26-46)
//   - selectModeInteractive()  (:157-197)  — blocking startup mode picker
//   - cycleMode()  (:203-207)  and handleRuntimeKey()  (:209-268)
//   - the main-loop mode-change detection condition  (:445-455)
//
// Behavior is preserved byte-for-byte: key mappings, mode cycling, MANUAL
// adjust keys (w/s/a/d/m), the blocking startup picker, and the TDC/PID-only
// mode-change reset condition are unchanged. Only "where the state lives"
// changes (file-scope globals + free functions -> class members + methods).
//
// ⚠️ selectModeInteractive() stays blocking here (Task-7 replaces it with a
// param-driven path; this task only relocates it).
//
// ROS-free: depends only on termios/unistd (raw terminal) + albc_kinematics.h
// for the MANUAL step constants and DEG/RAD helpers used by callers. No
// ros/ros.h, no std_msgs.

#ifndef ALBC_CONTROL_MODE_MANAGER_H
#define ALBC_CONTROL_MODE_MANAGER_H

#include <cstdio>
#include <termios.h>
#include <unistd.h>

namespace albc {

// ---- Control mode enums (former albc_controller.cpp:52-68) -----------------

enum class ControlMode : int { TDC = 1, PID = 2, FIXED = 3, MANUAL = 4 };
// TDC: Simplified Time-Delay Control (currently incremental PD with buoyancy compensation)
// PID: Standard PID with separate roll/pitch gains
// FIXED: Fixed end-effector position FK(90°,90°) for testing
// MANUAL: Direct joint angle or EE position control for calibration/testing

enum class ManualSubMode { JOINT, POSITION };

inline const char* controlModeName(ControlMode m) {
    switch (m) {
        case ControlMode::TDC:    return "TDC";
        case ControlMode::PID:    return "PID";
        case ControlMode::FIXED:  return "FIXED";
        case ControlMode::MANUAL: return "MANUAL";
        default:                  return "???";
    }
}

class ModeManager {
public:
    // MANUAL adjust steps (former albc_controller.cpp:77-78).
    static constexpr double MANUAL_ANGLE_STEP = 5.0;   // deg per keypress
    static constexpr double MANUAL_POS_STEP   = 0.01;  // m per keypress

    // ---- Terminal keyboard (raw mode, former :26-46) -----------------------

    // Switch STDIN to non-blocking raw mode for runtime key polling.
    // Uses initial_term_settings_ saved in selectModeInteractive().
    void initKeyboard() {
        struct termios raw = initial_term_settings_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;   // non-blocking
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    // Restore the original terminal settings (atexit handler).
    void closeKeyboard() {
        tcsetattr(STDIN_FILENO, TCSANOW, &initial_term_settings_);
    }

    // Non-blocking single-byte read; returns -1 if no key is pending.
    int readKey() {
        char ch;
        if (read(STDIN_FILENO, &ch, 1) == 1)
            return ch;
        return -1;
    }

    // ---- Startup mode selection (blocking, former :157-197) ----------------

    // Print the menu and block until the user presses 1/2/3/4, then switch to
    // non-blocking raw mode. Sets control_mode_ and returns it.
    // Byte-identical to the former selectModeInteractive().
    ControlMode selectModeInteractive() {
        // Save original terminal settings before any modification
        tcgetattr(STDIN_FILENO, &initial_term_settings_);

        printf("\033[2J\033[H");
        printf("═══════════════════════════════════════════════════\n");
        printf("        ALBC Controller - Mode Selection\n");
        printf("═══════════════════════════════════════════════════\n");
        printf("  [1] TDC    - Time-Delay Control\n");
        printf("  [2] PID    - PID Control\n");
        printf("  [3] FIXED  - Fixed Position (test)\n");
        printf("  [4] MANUAL - Manual Position\n");
        printf("═══════════════════════════════════════════════════\n");
        printf(" Select mode (1/2/3/4): ");
        fflush(stdout);

        // Blocking read for mode selection
        struct termios blocking = initial_term_settings_;
        blocking.c_lflag &= ~(ICANON | ECHO);
        blocking.c_cc[VMIN] = 1;   // block until 1 char
        blocking.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &blocking);

        ControlMode selected = ControlMode::TDC;
        while (true) {
            char ch;
            if (read(STDIN_FILENO, &ch, 1) == 1) {
                if (ch >= '1' && ch <= '4') {
                    selected = static_cast<ControlMode>(ch - '0');
                    printf("%c\n\n Starting [%s] mode...\n", ch, controlModeName(selected));
                    fflush(stdout);
                    usleep(500000);  // brief pause to show selection
                    break;
                }
            }
        }

        // Switch to non-blocking for runtime
        initKeyboard();
        control_mode_ = selected;
        return selected;
    }

    // ---- Runtime key handling (former :203-268) ----------------------------

    // Cycle mode TDC→PID→FIXED→MANUAL→TDC (former cycleMode(), :203-207).
    void cycleMode() {
        int m = static_cast<int>(control_mode_);
        m = (m % 4) + 1;  // 1→2→3→4→1
        control_mode_ = static_cast<ControlMode>(m);
    }

    // Apply a runtime key. Byte-identical to the former handleRuntimeKey():
    // '=' cycles, '1'-'4' select directly, w/s/a/d/m adjust MANUAL state.
    void handleRuntimeKey(int ch) {
        switch (ch) {
        case '=':   // Cycle mode: TDC→PID→FIXED→TDC
            cycleMode();
            break;
        case '1':   // Direct select: TDC
            control_mode_ = ControlMode::TDC;
            break;
        case '2':   // Direct select: PID
            control_mode_ = ControlMode::PID;
            break;
        case '3':   // Direct select: FIXED
            control_mode_ = ControlMode::FIXED;
            break;
        case '4':   // Direct select: MANUAL
            control_mode_ = ControlMode::MANUAL;
            break;
        // Manual mode keys (only active in MANUAL mode)
        case 'w':
            if (control_mode_ == ControlMode::MANUAL) {
                if (manual_submode_ == ManualSubMode::JOINT)
                    manual_theta1_deg_ += MANUAL_ANGLE_STEP;
                else
                    manual_y_ += MANUAL_POS_STEP;
            }
            break;
        case 's':
            if (control_mode_ == ControlMode::MANUAL) {
                if (manual_submode_ == ManualSubMode::JOINT)
                    manual_theta1_deg_ -= MANUAL_ANGLE_STEP;
                else
                    manual_y_ -= MANUAL_POS_STEP;
            }
            break;
        case 'a':
            if (control_mode_ == ControlMode::MANUAL) {
                if (manual_submode_ == ManualSubMode::JOINT)
                    manual_theta2_deg_ -= MANUAL_ANGLE_STEP;
                else
                    manual_x_ -= MANUAL_POS_STEP;
            }
            break;
        case 'd':
            if (control_mode_ == ControlMode::MANUAL) {
                if (manual_submode_ == ManualSubMode::JOINT)
                    manual_theta2_deg_ += MANUAL_ANGLE_STEP;
                else
                    manual_x_ += MANUAL_POS_STEP;
            }
            break;
        case 'm':
            if (control_mode_ == ControlMode::MANUAL) {
                manual_submode_ = (manual_submode_ == ManualSubMode::JOINT)
                                 ? ManualSubMode::POSITION : ManualSubMode::JOINT;
            }
            break;
        default:
            break;
        }
    }

    // ---- Mode-change detection (former main-loop block :445-455) -----------

    // Returns true exactly once after the control mode changes into TDC or PID
    // (the only modes that need a feedback-state reset). The caller performs
    // the actual reset (forwardKinematics + attitude.resetForModeChange).
    // Byte-identical to the former condition:
    //   if (control_mode != prev_mode) { if (TDC||PID) {…}; prev_mode = …; }
    // The prev_mode latch updates on EVERY mode change (not just TDC/PID), so a
    // FIXED→MANUAL change is consumed without re-firing — exactly as the source.
    bool detectModeChange() {
        bool needs_reset = false;
        if (control_mode_ != prev_mode_) {
            if (control_mode_ == ControlMode::TDC || control_mode_ == ControlMode::PID) {
                needs_reset = true;
            }
            prev_mode_ = control_mode_;
        }
        return needs_reset;
    }

    // ---- Mode setter (former reconfigureCallback control_mode write) -------

    // Set the active mode from an external source (dynamic_reconfigure).
    // Mirrors reconfigureCallback's `control_mode = static_cast<ControlMode>(…)`.
    void setMode(ControlMode m) { control_mode_ = m; }

    // Latch prev_mode_ to the current mode. Call once just before entering the
    // control loop to reproduce the former `static ControlMode prev_mode =
    // control_mode;` initializer (albc_controller.cpp:445), so the first tick
    // never fires a mode-change reset regardless of the selected/reconfigured
    // mode.
    void syncPrevMode() { prev_mode_ = control_mode_; }

    // ---- Manual-state setters (former reconfigure + param load writes) -----

    void setManualState(double theta1_deg, double theta2_deg, double x, double y) {
        manual_theta1_deg_ = theta1_deg;
        manual_theta2_deg_ = theta2_deg;
        manual_x_ = x;
        manual_y_ = y;
    }

    // ---- Getters -----------------------------------------------------------

    ControlMode  mode()          const { return control_mode_; }
    ManualSubMode manualSubmode() const { return manual_submode_; }
    double manualTheta1() const { return manual_theta1_deg_; }
    double manualTheta2() const { return manual_theta2_deg_; }
    double manualX()      const { return manual_x_; }
    double manualY()      const { return manual_y_; }

private:
    ControlMode control_mode_ = ControlMode::TDC;
    ManualSubMode manual_submode_ = ManualSubMode::JOINT;
    double manual_theta1_deg_ = 90.0, manual_theta2_deg_ = 90.0;
    double manual_x_ = 0.0, manual_y_ = 0.0;

    // Mode-change latch (former main-loop `static ControlMode prev_mode`).
    ControlMode prev_mode_ = ControlMode::TDC;

    // Saved terminal settings (former file-scope initial_term_settings).
    struct termios initial_term_settings_ {};
};

}  // namespace albc

#endif  // ALBC_CONTROL_MODE_MANAGER_H
