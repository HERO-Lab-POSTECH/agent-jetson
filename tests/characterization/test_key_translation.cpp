// Characterization test: V3 key translation layer
//
// Pins the CURRENT behavior of agent_main.cpp's key_input_callback (the V3
// translation switch). This is a SAFETY NET for the aggressive redesign:
// the redesigned code must produce byte-identical (command, translated)
// outputs for every input key under every toggle state.
//
// Oracle (translate_key) is a pure, ROS-free extraction of the switch in
// agent_main.cpp:207-298. Each translation produces up to two outputs:
//   - cmd:        char published to /hero_agent/command  (Arduino), or 0 if none
//   - translated: char published to /hero_agent/key_translated (Jetson), or 0 if none
// 0 (NUL) is the "not published" sentinel; no real key maps to 0.
//
// Toggle state (relay/yaw/depth/laser) is supplied explicitly because the
// real code reads it from /hero_agent/state bits — we pin both branches.
//
// Build & run (local, no ROS needed):
//   c++ -std=c++11 -I. tests/characterization/test_key_translation.cpp -o /tmp/t && /tmp/t

#include "../../robot/hero_agent/include/hero_agent/key_translator.h"
#include <cstdio>
#include <cstring>

using namespace hero;

static int failures = 0;
static int checks = 0;

static void expect_xlate(const ToggleState& st, int key,
                         char want_cmd, char want_tr, const char* desc)
{
    checks++;
    KeyXlate got = translate_key(key, st);
    if (got.cmd != want_cmd || got.translated != want_tr) {
        failures++;
        std::printf("FAIL [%s] key=%d('%c'): got (cmd=%d,tr=%d) want (cmd=%d,tr=%d)\n",
                    desc, key, (key >= 32 && key < 127) ? key : '?',
                    got.cmd, got.translated, want_cmd, want_tr);
    }
}

int main()
{
    // Toggle states. Fields: {relay, yaw, depth, laser}
    ToggleState off = {0, 0, 0, 0};   // all hardware toggles reported OFF
    ToggleState on  = {1, 1, 1, 1};   // all hardware toggles reported ON

    // ── Number row: hardware toggles, OFF → "turn ON" char to Arduino ──
    expect_xlate(off, '1', 'e', 0, "relay OFF->ON sends 'e'");
    expect_xlate(off, '2', 'y', 0, "yaw   OFF->ON sends 'y'");
    expect_xlate(off, '3', 'p', 0, "depth OFF->ON sends 'p'");
    expect_xlate(off, '5', 'r', 0, "laser OFF->ON sends 'r'");

    // ── Number row: hardware toggles, ON → "turn OFF" char to Arduino ──
    expect_xlate(on, '1', 't', 0, "relay ON->OFF sends 't'");
    expect_xlate(on, '2', 'h', 0, "yaw   ON->OFF sends 'h'");
    expect_xlate(on, '3', ';', 0, "depth ON->OFF sends ';'");
    expect_xlate(on, '5', 'f', 0, "laser ON->OFF sends 'f'");

    // '4' PWM Init: always 'g' to Arduino, state-independent
    expect_xlate(off, '4', 'g', 0, "PWM init -> 'g' (off state)");
    expect_xlate(on,  '4', 'g', 0, "PWM init -> 'g' (on state)");

    // ── Jetson-only letter keys ──
    expect_xlate(off, 'r', 0, 'r', "heave up   -> translated 'r' only");
    expect_xlate(off, 'f', 0, 'f', "heave down -> translated 'f' only");

    // ── Arduino-only letter keys ──
    expect_xlate(off, 'N', 'n', 0, "yaw reset  -> command 'n' only");
    expect_xlate(off, 'o', 'o', 0, "depth -0.1 -> command 'o' only");

    // ── Blocked keys: no output at all ──
    const char* blocked = ";nm.,tgyheq67890p";
    for (const char* k = blocked; *k; ++k)
        expect_xlate(off, *k, 0, 0, "blocked key -> no output");

    // ── Rosbag toggle 'R': internal flag, publishes nothing ──
    expect_xlate(off, 'R', 0, 0, "rosbag toggle 'R' -> no published output");

    // ── Pass-through keys: same char to BOTH topics ──
    const char* passthru = "wsadzxujikl cvb";  // (space ignored below)
    for (const char* k = passthru; *k; ++k) {
        if (*k == ' ') continue;
        expect_xlate(off, *k, *k, *k, "pass-through -> both topics");
    }

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
