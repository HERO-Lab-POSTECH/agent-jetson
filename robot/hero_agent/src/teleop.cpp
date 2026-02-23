#include "hero_agent/hero_agent_types.h"

#include <termios.h>
#include <unistd.h>
#include <cstdio>

// ==============================
// Terminal keyboard
// ==============================

static struct termios initial_settings, new_settings;
static int peek_character = -1;

// Fallback timeout: if agent_main is not running, process key directly
// after waiting this many loop iterations (100ms at 100Hz = 10 ticks)
static const int FALLBACK_TICKS = 10;
static int pending_key = -1;
static int fallback_counter = 0;

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
// Jetson-side key mapping (processKey):
//   Movement:  w/s=Surge  a/d=Sway  r/f=Heave
//   Lawnmower: p=Start  o=Stop
//   Darknet:   n=On  m=Off
//
// NOTE: This processKey receives TRANSLATED keys from agent_main
//       via /hero_agent/key_translated. The keys arriving here use
//       the original Jetson key codes (not V3 codes).
// ==============================

static void processKey(int ch, ros::Rate& loop_rate)
{
    if (ch < 0) return;

    switch (ch) {

    // --- XYZ teleop ---
    case 'w': target.x += teleop_xy_step;  break;
    case 's': target.x -= teleop_xy_step;  break;
    case 'd': target.y += teleop_xy_step;  break;
    case 'a': target.y -= teleop_xy_step;  break;
    case 'r': target.z -= teleop_z_step;   break;
    case 'f': target.z += teleop_z_step;   break;

    // --- Lawnmower control ---
    case 'o': ctrl.lawnmower = 0; break;
    case 'p':
        ctrl.lawnmower = 1;
        lawnmower.sway_count = 0;
        lawnmower.surge_count = 0;
        break;

    // --- DARKNET control ---
    case 'n': ctrl.darknet = 1; break;
    case 'm': ctrl.darknet = 0; break;

    default:
        break;
    }
}

// ==============================
// Main handler (called from agent_command main loop)
//
// V3 architecture:
//   - Stdin keys → publish to /hero_agent/key_input (for agent_main to translate)
//   - Queue reads from /hero_agent/key_translated (already translated by agent_main)
//   - Fallback: if no translated key arrives within 100ms, process stdin key directly
//     (standalone mode without agent_main)
// ==============================

void handleKeyboardInput(ros::Rate& loop_rate)
{
    int ch = -1;

    // Check stdin first (for rosrun usage)
    if (_kbhit()) {
        int raw_key = _getch();
        // Publish to /hero_agent/key_input for agent_main translation
        std_msgs::Int8 key_msg;
        key_msg.data = raw_key;
        pub_key_input.publish(key_msg);
        // Start fallback timer (in case agent_main is not running)
        pending_key = raw_key;
        fallback_counter = 0;
    }

    // Check translated topic queue (from agent_main)
    if (!key_input_queue.empty()) {
        ch = key_input_queue.front();
        key_input_queue.pop();
        // Got translated key, cancel fallback
        pending_key = -1;
        fallback_counter = 0;
    }
    // Fallback: process raw key directly if agent_main didn't respond
    else if (pending_key >= 0) {
        fallback_counter++;
        if (fallback_counter >= FALLBACK_TICKS) {
            ch = pending_key;
            pending_key = -1;
            fallback_counter = 0;
        }
    }

    if (ch < 0) return;
    processKey(ch, loop_rate);
}
