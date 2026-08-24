// Arduino.h — HOST SHIM, tests only. Not the real Arduino core.
//
// WHY THIS EXISTS
// ---------------
// The firmware headers (thrusters.h, ahrs.h) `#include <Arduino.h>`, so a host
// compiler cannot touch firmware/agent/*.cpp without one. This shim supplies the
// two things pid.cpp actually needs from the core -- the `constrain` macro and
// the `boolean` typedef -- and nothing else.
//
// It lives here rather than in firmware/ on purpose: firmware/ must stay exactly
// what the board compiles, and the board has the real Arduino.h. run.sh already
// passes `-I.` (this directory) and CMakeLists.txt already adds
// ${CMAKE_CURRENT_SOURCE_DIR}, so both test paths pick it up with no build-config
// change. Nothing outside tests/characterization/ can see it.
//
// The `constrain` definition below is COPIED from the Arduino core (wiring.h /
// Arduino.h, unchanged since 1.0) rather than rewritten -- a macro that evaluates
// its argument three times is part of the behaviour under test.
#ifndef AGENT_TEST_ARDUINO_SHIM_H
#define AGENT_TEST_ARDUINO_SHIM_H

#include <stdint.h>
#include <stddef.h>

typedef bool boolean;
typedef uint8_t byte;

// Arduino core, verbatim. Triple-evaluates `amt` -- deliberately preserved.
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif
#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#endif // AGENT_TEST_ARDUINO_SHIM_H
