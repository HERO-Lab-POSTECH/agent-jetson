#include "hero_agent/agent_command_types.h"

// Forward declaration
extern void initRecoveryConfigs();

// ==============================
// Keyboard Teleop Handler
// ==============================

void handleKeyboardInput(ros::Rate& loop_rate)
{
    int ch = -1;

    // Check stdin (rosrun mode)
    if (_kbhit()) {
        ch = _getch();
        _putch(ch);
    }
    // Check topic queue (roslaunch + teleop mode)
    else if (!key_input_queue.empty()) {
        ch = key_input_queue.front();
        key_input_queue.pop();
    }

    if (ch < 0) return;
    msg_target.command = 0;

    switch (ch) {

    // --- TDC toggle ---
    case ',':
        resetQrErrors();
        cont_qr_tdc = 1;
        break;
    case '.':
        cont_qr_tdc = 0;
        break;

    // --- TDC parameter tuning ---
    case 'y': qr_Mb  += 0.1;     break;
    case 'h': qr_Mb  -= 0.1;     break;
    case 'u': qr_KKp += 0.00001; break;
    case 'i': qr_KKv += 0.001;   break;

    // --- Auto recovery sequence ---
    case 't':
        start_recovery = 1;
        break;
    case 'g':
        start_recovery = 0;
        start_count = 0;
        start_step = 0;
        start_step_pre = 0;
        break;

    // --- Target reset ---
    case 'q':
        target_x = 0; target_y = 0; target_z = 0; target_yaw = 0;
        qr_based_calibration = 0;
        break;

    // --- Send current target to controller ---
    case 'e':
        msg_target.command = 1;
        msg_target.TARGET_X = target_x;
        msg_target.TARGET_Y = target_y;
        msg_target.TARGET_Z = target_z;
        pub_target.publish(msg_target);
        loop_rate.sleep();
        ros::spinOnce();
        break;

    // --- Winch calibrate ---
    case '1':
        calib_position = current_position;
        target_position = current_position;
        target_meter = 0;
        break;
    case '2':
        if (target_meter < 20) target_meter++;
        break;
    case '3':
        if (target_meter > 0) target_meter--;
        break;
    case '4':
        target_position += teleop_winch_step;
        msg_winch_target.data = target_position;
        pub_winch_target.publish(msg_winch_target);
        break;
    case '5':
        target_position -= teleop_winch_step;
        msg_winch_target.data = target_position;
        pub_winch_target.publish(msg_winch_target);
        break;

    // --- XYZ teleop ---
    case 'w': target_x += teleop_xy_step;  break;
    case 's': target_x -= teleop_xy_step;  break;
    case 'd': target_y += teleop_xy_step;  break;
    case 'a': target_y -= teleop_xy_step;  break;
    case 'r': target_z -= teleop_z_step;   break;
    case 'f': target_z += teleop_z_step;   break;

    // --- Experiment recording ---
    case '[':
        start_record = 1;
        time_count = 0;
        cont_qr_tdc = 0;
        cont_recovery = -1;
        flag_count = 0;
        resetQrErrors();
        break;

    // --- QR recovery mode selection ---
    case '/':
        cont_recovery = -1;
        flag_count = 0;
        resetQrErrors();
        break;
    case ']':
        cont_recovery = -2;
        flag_count = 0;
        resetQrErrors();
        break;
    case 'z':
        cont_recovery = 1;
        flag_count = 0; qr_count = 0;
        qr_x_error_sum = 0; qr_z_error_sum = 0;
        break;
    case 'x':
        cont_recovery = 2;
        qr_based_calibration = 0;
        flag_count = 0; qr_count = 0;
        qr_x_error_sum = 0; qr_z_error_sum = 0;
        break;
    case 'c':
        cont_recovery = 3;
        flag_count = 0; qr_count = 0;
        qr_x_error_sum = 0; qr_z_error_sum = 0;
        break;
    case 'v':
        cont_recovery = 4;
        flag_count = 0; qr_count = 0;
        qr_x_error_sum = 0; qr_z_error_sum = 0;
        break;
    case 'b':
        cont_recovery = 0;
        flag_count = 0;
        qr_based_calibration = 0;
        qr_count = 0;
        qr_x_error_sum = 0; qr_z_error_sum = 0;
        break;

    // --- Mosaic control ---
    case 'o': cont_mosaic = 0; break;
    case 'p': cont_mosaic = 1; break;

    // --- DARKNET control ---
    case 'n': cont_darknet = 1; break;
    case 'm': cont_darknet = 0; break;

    default:
        break;
    }
}
