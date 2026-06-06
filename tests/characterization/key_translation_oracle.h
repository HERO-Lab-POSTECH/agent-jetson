// Golden oracle: V3 key translation layer (ROS-free extraction)
//
// 1:1 transcription of agent_main.cpp:207-298 (key_input_callback) with the
// ROS plumbing (publish calls, debounce timing) factored out. The redesign
// must keep translate_key() byte-identical, or the characterization tests
// in this directory will catch the behavior change.
//
// Mapping of the real code to this oracle:
//   send_command(c)   -> result.cmd = c        (goes to /hero_agent/command)
//   send_translated(c)-> result.translated = c (goes to /hero_agent/key_translated)
//   not published     -> 0 (NUL sentinel)
//
// Debounce (agent_main.cpp:73) is a 500ms time gate; it does not change WHICH
// char is sent, only WHETHER a repeat within 500ms is dropped. The oracle
// assumes the debounce gate is OPEN (first press / >500ms apart), which is the
// behavior the redesign must preserve for the char mapping itself. Debounce
// timing is tested separately if needed.
//
// 'R' (rosbag toggle, agent_main.cpp:287) flips an internal flag and publishes
// nothing — oracle returns {0,0}. lawnmower_on for key 'p' is a stateful
// internal toggle; the oracle takes its prior value explicitly.

#ifndef KEY_TRANSLATION_ORACLE_H
#define KEY_TRANSLATION_ORACLE_H

// Hardware toggle state as last reported by Arduino via /hero_agent/state
// (agent_main.cpp:139-142 decode control_flags bits 0..3).
struct ToggleState {
    int relay_enabled;          // bit 2
    int control_yaw_enabled;    // bit 0
    int control_depth_enabled;  // bit 1
    int laser_enabled;          // bit 3
};

struct KeyXlate {
    char cmd;         // -> /hero_agent/command (Arduino), 0 = none
    char translated;  // -> /hero_agent/key_translated (Jetson), 0 = none
};

// Pure transcription of agent_main.cpp:207-298.
// lawnmower_on_prev: current value of the internal lawnmower_on flag before
//   this keypress (only consulted for key 'p'). Defaults to false.
inline KeyXlate translate_key(int ch, const ToggleState& st,
                              bool lawnmower_on_prev = false)
{
    KeyXlate out = {0, 0};

    switch (ch) {

    // ── Number Row: Hardware Toggles ── (agent_main.cpp:215-237)
    case '1':  // Toggle Relay
        out.cmd = st.relay_enabled ? 't' : 'e';
        break;
    case '2':  // Toggle Yaw
        out.cmd = st.control_yaw_enabled ? 'h' : 'y';
        break;
    case '3':  // Toggle Depth
        out.cmd = st.control_depth_enabled ? ';' : 'p';
        break;
    case '4':  // PWM Init -> Arduino only
        out.cmd = 'g';
        break;
    case '5':  // Toggle Laser
        out.cmd = st.laser_enabled ? 'f' : 'r';
        break;

    // ── Letter Keys: Jetson-Only ── (agent_main.cpp:241-253)
    case 'r':  // Heave Up
        out.translated = 'r';
        break;
    case 'f':  // Heave Down
        out.translated = 'f';
        break;
    case 'p':  // Toggle Lawnmower (Jetson only)
        // lawnmower_on = !lawnmower_on; send_translated(lawnmower_on ? 'p':'o')
        out.translated = (!lawnmower_on_prev) ? 'p' : 'o';
        break;

    // ── Letter Keys: Arduino-Only ── (agent_main.cpp:257-262)
    case 'N':  // Yaw Reset -> Arduino 'n'
        out.cmd = 'n';
        break;
    case 'o':  // Depth -0.1 -> Arduino 'o'
        out.cmd = 'o';
        break;

    // ── Blocked Keys (freed, no function) ── (agent_main.cpp:267-283)
    case ';': case 'n': case 'm': case '.': case ',':
    case 't': case 'g': case 'y': case 'h':
    case 'e': case 'q':
    case '6': case '7': case '8': case '9': case '0':
        break;  // {0, 0}

    // ── Rosbag Toggle (agent_main internal) ── (agent_main.cpp:287-289)
    case 'R':
        break;  // {0, 0} — flips record flag, publishes nothing

    // ── Pass-Through: Both Arduino + Jetson ── (agent_main.cpp:293-295)
    // w/s/a/d, z/x, u/j, i/k, l, c/v/b
    default:
        out.cmd = (char)ch;
        out.translated = (char)ch;
        break;
    }

    return out;
}

#endif // KEY_TRANSLATION_ORACLE_H
