// Unit test: declarative keymap table (keymap.h)
//
// Verifies the V3 keymap moved from code (giant switch) into data (KEYMAP[]).
// Toggle keys carry cmd_off/cmd_on + toggle_bit; blocked keys and pass-through
// keys are both absent from the table — lookup_key returns nullptr for them, and
// is_blocked_key separates the two cases. The 'p' key is intentionally NOT in
// the table (its former survey-mode function was removed).
//
// Build & run (local, no ROS):
//   c++ -std=c++11 -Wall -I. test_keymap_table.cpp -o /tmp/tk && /tmp/tk

#include "../../robot/hero_agent/include/hero_agent/keymap.h"
#include <cstdio>

using namespace hero;

static int failures = 0, checks = 0;

static void expect_eq_int(int got, int want, const char* desc)
{
    checks++;
    if (got != want) { failures++; std::printf("FAIL [%s]: got %d want %d\n", desc, got, want); }
}
static void expect_true(bool got, const char* desc)
{
    checks++;
    if (!got) { failures++; std::printf("FAIL [%s]: got false want true\n", desc); }
}
static void expect_null(const KeyDef* got, const char* desc)
{
    checks++;
    if (got != nullptr) { failures++; std::printf("FAIL [%s]: got non-null want nullptr\n", desc); }
}
static void expect_nonnull(const KeyDef* got, const char* desc)
{
    checks++;
    if (got == nullptr) { failures++; std::printf("FAIL [%s]: got nullptr want non-null\n", desc); }
}

int main()
{
    // (1) '1' Relay toggle
    {
        const KeyDef* k = lookup_key('1');
        expect_nonnull(k, "'1' present");
        if (k) {
            expect_true(k->toggle, "'1' toggle==true");
            expect_eq_int(k->cmd_off, 'e', "'1' cmd_off=='e'");
            expect_eq_int(k->cmd_on, 't', "'1' cmd_on=='t'");
            expect_true(k->toggle_bit == ToggleBit::RELAY, "'1' bit==RELAY");
        }
    }

    // (2) '2' Yaw, '3' Depth, '5' Laser toggles
    {
        const KeyDef* k = lookup_key('2');
        expect_nonnull(k, "'2' present");
        if (k) {
            expect_eq_int(k->cmd_off, 'y', "'2' cmd_off=='y'");
            expect_eq_int(k->cmd_on, 'h', "'2' cmd_on=='h'");
            expect_true(k->toggle_bit == ToggleBit::YAW, "'2' bit==YAW");
        }
    }
    {
        const KeyDef* k = lookup_key('3');
        expect_nonnull(k, "'3' present");
        if (k) {
            expect_eq_int(k->cmd_off, 'p', "'3' cmd_off=='p'");
            expect_eq_int(k->cmd_on, ';', "'3' cmd_on==';'");
            expect_true(k->toggle_bit == ToggleBit::DEPTH, "'3' bit==DEPTH");
        }
    }
    {
        const KeyDef* k = lookup_key('5');
        expect_nonnull(k, "'5' present");
        if (k) {
            expect_eq_int(k->cmd_off, 'r', "'5' cmd_off=='r'");
            expect_eq_int(k->cmd_on, 'f', "'5' cmd_on=='f'");
            expect_true(k->toggle_bit == ToggleBit::LASER, "'5' bit==LASER");
        }
    }

    // (3) Fixed cmd keys: '4'->'g'(debounce), 'N'->'n', 'o'->'o'
    {
        const KeyDef* k = lookup_key('4');
        expect_nonnull(k, "'4' present");
        if (k) {
            expect_eq_int(k->cmd_off, 'g', "'4' cmd_off=='g'");
            expect_true(k->debounce, "'4' debounce==true");
        }
    }
    {
        const KeyDef* k = lookup_key('N');
        expect_nonnull(k, "'N' present");
        if (k) expect_eq_int(k->cmd_off, 'n', "'N' cmd_off=='n'");
    }
    {
        const KeyDef* k = lookup_key('o');
        expect_nonnull(k, "'o' present");
        if (k) expect_eq_int(k->cmd_off, 'o', "'o' cmd_off=='o'");
    }

    // (4) Fixed translated keys: 'r'->translated 'r' (cmd_off 0), 'f'->translated 'f'
    {
        const KeyDef* k = lookup_key('r');
        expect_nonnull(k, "'r' present");
        if (k) {
            expect_eq_int(k->translated, 'r', "'r' translated=='r'");
            expect_eq_int(k->cmd_off, 0, "'r' cmd_off==0");
        }
    }
    {
        const KeyDef* k = lookup_key('f');
        expect_nonnull(k, "'f' present");
        if (k) expect_eq_int(k->translated, 'f', "'f' translated=='f'");
    }

    // (5) pass-through key 'w': absent from table, NOT blocked
    expect_null(lookup_key('w'), "'w' lookup==nullptr (pass-through)");
    expect_true(!is_blocked_key('w'), "'w' is_blocked==false");

    // (6) blocked keys
    expect_true(is_blocked_key(';'), "';' is_blocked==true");
    expect_true(is_blocked_key('R'), "'R' is_blocked==true");
    expect_true(is_blocked_key('6'), "'6' is_blocked==true");

    // (7) lawnmower 'p' removed: absent from table, now a blocked key
    expect_null(lookup_key('p'), "'p' lookup==nullptr (lawnmower removed)");
    expect_true(is_blocked_key('p'), "'p' is_blocked==true (no function)");

    // (8) debounce flags: '1'/'2'/'3'/'4'/'5' true; 'N'/'o'/'r'/'f' false
    {
        const char on[]  = {'1', '2', '3', '4', '5'};
        const char off[] = {'N', 'o', 'r', 'f'};
        for (char c : on) {
            const KeyDef* k = lookup_key(c);
            expect_nonnull(k, "debounce-on key present");
            if (k) { char d[32]; std::snprintf(d, sizeof d, "'%c' debounce==true", c); expect_true(k->debounce, d); }
        }
        for (char c : off) {
            const KeyDef* k = lookup_key(c);
            expect_nonnull(k, "debounce-off key present");
            if (k) { char d[32]; std::snprintf(d, sizeof d, "'%c' debounce==false", c); expect_true(!k->debounce, d); }
        }
    }

    std::printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
