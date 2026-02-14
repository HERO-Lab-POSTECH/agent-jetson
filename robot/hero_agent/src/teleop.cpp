#include "hero_agent/hero_agent_types.h"

#include <termios.h>
#include <sys/select.h>
#include <unistd.h>

// ==============================
// Keyboard Teleop Handler
// ==============================
// Dual-input: reads from /hero_agent/key_input topic queue AND stdin.
// stdin allows direct control from the roslaunch terminal (SSH, no xterm).

static struct termios original_termios;
static bool terminal_initialized = false;

void init_keyboard()
{
    if (terminal_initialized) return;
    struct termios raw;
    tcgetattr(STDIN_FILENO, &original_termios);
    raw = original_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    terminal_initialized = true;
}

void close_keyboard()
{
    if (!terminal_initialized) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
    terminal_initialized = false;
}

static int readStdinKey()
{
    fd_set fds;
    struct timeval tv = {0, 0};
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
        unsigned char c;
        if (read(STDIN_FILENO, &c, 1) == 1) return c;
    }
    return -1;
}

static void processKey(int ch, ros::Rate& loop_rate)
{
    if (ch < 0) return;
    msg_target.command = 0;

    switch (ch) {

    // --- TDC toggle ---
    case ',':
        resetQrErrors();
        ctrl.qr_tdc = 1;
        break;
    case '.':
        ctrl.qr_tdc = 0;
        break;

    // --- TDC parameter tuning ---
    case 'y': qr_gains.Mb  += 0.1;     break;
    case 'h': qr_gains.Mb  -= 0.1;     break;
    case 'u': qr_gains.KKp += 0.00001; break;
    case 'i': qr_gains.KKv += 0.001;   break;

    // --- Auto recovery sequence ---
    case 't':
        auto_recovery.active = 1;
        break;
    case 'g':
        auto_recovery.active = 0;
        auto_recovery.count = 0;
        auto_recovery.step = 0;
        auto_recovery.step_pre = 0;
        break;

    // --- Target reset ---
    case 'q':
        target.x = 0; target.y = 0; target.z = 0; target.yaw = 0;
        winch.qr_based_calibration = 0;
        break;

    // --- Send current target to controller ---
    case 'e':
        msg_target.command = 1;
        msg_target.TARGET_X = target.x;
        msg_target.TARGET_Y = target.y;
        msg_target.TARGET_Z = target.z;
        pub_target.publish(msg_target);
        loop_rate.sleep();
        break;

    // --- Winch calibrate ---
    case '1':
        winch.calib_position = winch.current_position;
        winch.target_position = winch.current_position;
        winch.target_meter = 0;
        break;
    case '2':
        if (winch.target_meter < 20) winch.target_meter++;
        break;
    case '3':
        if (winch.target_meter > 0) winch.target_meter--;
        break;
    case '4':
        winch.target_position += teleop_winch_step;
        msg_winch_target.data = winch.target_position;
        pub_winch_target.publish(msg_winch_target);
        break;
    case '5':
        winch.target_position -= teleop_winch_step;
        msg_winch_target.data = winch.target_position;
        pub_winch_target.publish(msg_winch_target);
        break;

    // --- XYZ teleop ---
    case 'w': target.x += teleop_xy_step;  break;
    case 's': target.x -= teleop_xy_step;  break;
    case 'd': target.y += teleop_xy_step;  break;
    case 'a': target.y -= teleop_xy_step;  break;
    case 'r': target.z -= teleop_z_step;   break;
    case 'f': target.z += teleop_z_step;   break;

    // --- Experiment recording ---
    case '[':
        start_record = 1;
        time_count = 0;
        ctrl.qr_tdc = 0;
        ctrl.recovery = -1;
        qr.flag_count = 0;
        resetQrErrors();
        break;

    // --- QR recovery mode selection ---
    case '/':
        ctrl.recovery = -1;
        qr.flag_count = 0;
        resetQrErrors();
        break;
    case ']':
        ctrl.recovery = -2;
        qr.flag_count = 0;
        resetQrErrors();
        break;
    case 'z':
        ctrl.recovery = 1;
        qr.flag_count = 0; qr.count = 0;
        qr.x_error_sum = 0; qr.z_error_sum = 0;
        break;
    case 'x':
        ctrl.recovery = 2;
        winch.qr_based_calibration = 0;
        qr.flag_count = 0; qr.count = 0;
        qr.x_error_sum = 0; qr.z_error_sum = 0;
        break;
    case 'c':
        ctrl.recovery = 3;
        qr.flag_count = 0; qr.count = 0;
        qr.x_error_sum = 0; qr.z_error_sum = 0;
        break;
    case 'v':
        ctrl.recovery = 4;
        qr.flag_count = 0; qr.count = 0;
        qr.x_error_sum = 0; qr.z_error_sum = 0;
        break;
    case 'b':
        ctrl.recovery = 0;
        qr.flag_count = 0;
        winch.qr_based_calibration = 0;
        qr.count = 0;
        qr.x_error_sum = 0; qr.z_error_sum = 0;
        break;

    // --- Mosaic control ---
    case 'o': ctrl.mosaic = 0; break;
    case 'p': ctrl.mosaic = 1; break;

    // --- DARKNET control ---
    case 'n': ctrl.darknet = 1; break;
    case 'm': ctrl.darknet = 0; break;

    default:
        break;
    }
}

void handleKeyboardInput(ros::Rate& loop_rate)
{
    // 1) Process topic-based input (from key_teleop.py)
    while (!key_input_queue.empty()) {
        int ch = key_input_queue.front();
        key_input_queue.pop();
        processKey(ch, loop_rate);
    }

    // 2) Process stdin input (direct terminal typing)
    int ch = readStdinKey();
    if (ch >= 0) {
        processKey(ch, loop_rate);
    }
}
