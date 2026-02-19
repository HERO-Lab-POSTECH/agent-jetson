#include "hero_agent/hero_agent_types.h"

#include <termios.h>
#include <unistd.h>
#include <cstdio>

// ==============================
// Terminal keyboard
// ==============================

static struct termios initial_settings, new_settings;
static int peek_character = -1;

// Flag to ignore self-published messages in key_input_queue
static bool ignore_next_queue = false;

void init_keyboard()
{
    tcgetattr(0, &initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}

void close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);
}

static int _kbhit()
{
    unsigned char ch;
    int nread;

    if (peek_character != -1)
        return 1;

    new_settings.c_cc[VMIN] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0, &ch, 1);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);

    if (nread == 1) {
        peek_character = ch;
        return 1;
    }
    return 0;
}

static int _getch()
{
    char ch;

    if (peek_character != -1) {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0, &ch, 1);
    return ch;
}

// ==============================
// Key processing
//
// Key mapping v2 (2026-02-19):
//   Movement:  w/s=Surge  a/d=Sway  r/f=Heave
//   Target:    e=Send  q=Reset
//   Toggles:   1=TDC  2=Darknet  3=Mosaic  4=AutoRecovery
//   Winch:     z=Calib  x/c=Meter+/-  v/b=Step+/-
//   Recovery:  5=Off  6=Approach  7=Close  8=Final  9=Deploy
//              0=ExpHold  -=ExpClose
//   TDC Tune:  i/k=Mb+/-  j=KKp+  l=KKv+
//   Record:    [=Experiment  R=Rosbag(agent_main)
// ==============================

static void processKey(int ch, ros::Rate& loop_rate)
{
    if (ch < 0) return;

    switch (ch) {

    // --- Movement (WASD + RF) ---
    case 'w': target.x += teleop_xy_step;  break;  // Surge +
    case 's': target.x -= teleop_xy_step;  break;  // Surge -
    case 'a': target.y -= teleop_xy_step;  break;  // Sway -
    case 'd': target.y += teleop_xy_step;  break;  // Sway +
    case 'r': target.z -= teleop_z_step;   break;  // Heave + (up)
    case 'f': target.z += teleop_z_step;   break;  // Heave - (down)

    // --- Target control ---
    case 'e':  // Send current target
        msg_target.command = 1;
        msg_target.TARGET_X = target.x;
        msg_target.TARGET_Y = target.y;
        msg_target.TARGET_Z = target.z;
        pub_target.publish(msg_target);
        msg_target.command = 0;
        break;
    case 'q':  // Reset target
        target.x = 0; target.y = 0; target.z = 0; target.yaw = 0;
        winch.qr_based_calibration = 0;
        break;

    // --- Toggles (number row) ---
    case '1':  // TDC on/off
        if (ctrl.qr_tdc == 0) { resetQrErrors(); ctrl.qr_tdc = 1; }
        else { ctrl.qr_tdc = 0; }
        break;
    case '2':  // Darknet on/off
        ctrl.darknet = ctrl.darknet ? 0 : 1;
        break;
    case '3':  // Mosaic on/off
        if (ctrl.mosaic == 0) {
            ctrl.mosaic = 1;
            mosaic.sway_count = 0;
            mosaic.surge_count = 0;
        } else {
            ctrl.mosaic = 0;
        }
        break;
    case '4':  // Auto Recovery on/off
        if (auto_recovery.active == 0) {
            auto_recovery.active = 1;
        } else {
            auto_recovery.active = 0;
            auto_recovery.count = 0;
            auto_recovery.step = 0;
            auto_recovery.step_pre = 0;
        }
        break;

    // --- Winch (ZXCVB) ---
    case 'z':  // Calibrate (set current = zero)
        winch.calib_position = winch.current_position;
        winch.target_position = winch.current_position;
        winch.target_meter = 0;
        break;
    case 'x':  // Meter + (max 20)
        if (winch.target_meter < 20) winch.target_meter++;
        break;
    case 'c':  // Meter -
        if (winch.target_meter > 0) winch.target_meter--;
        break;
    case 'v':  // Step + (release)
        winch.target_position += teleop_winch_step;
        msg_winch_target.data = winch.target_position;
        pub_winch_target.publish(msg_winch_target);
        break;
    case 'b':  // Step - (retract)
        winch.target_position -= teleop_winch_step;
        msg_winch_target.data = winch.target_position;
        pub_winch_target.publish(msg_winch_target);
        break;

    // --- Recovery modes (5-9, 0, -) ---
    case '5':  // Recovery off
        ctrl.recovery = 0;
        qr.flag_count = 0;
        winch.qr_based_calibration = 0;
        qr.count = 0;
        qr.x_error_sum = 0; qr.z_error_sum = 0;
        break;
    case '6':  // Approach
        ctrl.recovery = 1;
        qr.flag_count = 0; qr.count = 0;
        qr.x_error_sum = 0; qr.z_error_sum = 0;
        break;
    case '7':  // Close
        ctrl.recovery = 2;
        winch.qr_based_calibration = 0;
        qr.flag_count = 0; qr.count = 0;
        qr.x_error_sum = 0; qr.z_error_sum = 0;
        break;
    case '8':  // Final
        ctrl.recovery = 3;
        qr.flag_count = 0; qr.count = 0;
        qr.x_error_sum = 0; qr.z_error_sum = 0;
        break;
    case '9':  // Deploy
        ctrl.recovery = 4;
        qr.flag_count = 0; qr.count = 0;
        qr.x_error_sum = 0; qr.z_error_sum = 0;
        break;
    case '0':  // ExpHold
        ctrl.recovery = -1;
        qr.flag_count = 0;
        resetQrErrors();
        break;
    case '-':  // ExpClose
        ctrl.recovery = -2;
        qr.flag_count = 0;
        resetQrErrors();
        break;

    // --- TDC tuning (IJKL cluster) ---
    case 'i': qr_gains.Mb  += 0.1;     break;  // Mb +
    case 'k': qr_gains.Mb  -= 0.1;     break;  // Mb -
    case 'j': qr_gains.KKp += 0.00001; break;  // KKp +
    case 'l': qr_gains.KKv += 0.001;   break;  // KKv +

    // --- Experiment recording ---
    case '[':
        start_record = 1;
        time_count = 0;
        ctrl.qr_tdc = 0;
        ctrl.recovery = -1;
        qr.flag_count = 0;
        resetQrErrors();
        break;

    default:
        break;
    }
}

// ==============================
// Main handler (called from agent_command main loop)
// ==============================

void handleKeyboardInput(ros::Rate& loop_rate)
{
    int ch = -1;

    // Check stdin first (for rosrun usage)
    if (_kbhit()) {
        ch = _getch();
        // Publish to /hero_agent/key_input so agent_main receives it
        std_msgs::Int8 key_msg;
        key_msg.data = ch;
        pub_key_input.publish(key_msg);
        // [BUG FIX] Mark to skip the self-published message from queue
        ignore_next_queue = true;
    }
    // Check topic queue (from key_teleop.py or external publishers)
    else if (!key_input_queue.empty()) {
        if (ignore_next_queue) {
            // Discard the self-published message
            key_input_queue.pop();
            ignore_next_queue = false;
        } else {
            ch = key_input_queue.front();
            key_input_queue.pop();
        }
    }

    if (ch < 0) return;
    processKey(ch, loop_rate);
}
