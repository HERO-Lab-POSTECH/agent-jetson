// Dynamixel motor configuration for joint_angle_command node.
//
// Constants extracted CHARACTER-FOR-CHARACTER from joint_angle_command.cpp
// (Phase 4 D6). Values are byte-identical to the source; only their LOCATION
// moved here. This node is the Dynamixel motor driver — it consumes joint
// angle commands published by albc_controller and drives the two MX-series
// servos via the Dynamixel SDK. Nothing in this header is shared with the
// albc_controller control math (that lives in feedback_filters.h etc.).
//
// Hardware-pinned values (do NOT "modernize" or retune without intent):
//   - Motor IDs 11 / 12 on a single RS-485 bus.
//   - Baud 57600, Protocol 2.0, udev alias /dev/ttyDynamixel.
//   - Control-table addresses are MX(Protocol 2.0) register offsets.
//   - 2048 ticks = pi rad (a half-turn) for the 4096-tick/rev encoder.
//   - Present-current LSB ~= 2.69 mA (CURRENT_TO_MA).

#ifndef ALBC_CONTROL_DYNAMIXEL_CONFIG_H
#define ALBC_CONTROL_DYNAMIXEL_CONFIG_H

#include <cmath>     // M_PI
#include <cstdint>   // fixed-width integer types

// ==============================
// Bus / device configuration
// ==============================

static constexpr uint8_t  JOINT1_ID   = 11;
static constexpr uint8_t  JOINT2_ID   = 12;
static constexpr int      BAUDRATE    = 57600;
static const char*        SERIAL_PORT = "/dev/ttyDynamixel";
static constexpr float    PROTOCOL    = 2.0;

// ==============================
// Control-table register addresses (MX, Protocol 2.0)
// ==============================

static constexpr uint16_t ADDR_TORQUE_ENABLE        = 64;
static constexpr uint16_t ADDR_POSITION_D_GAIN      = 80;
static constexpr uint16_t ADDR_POSITION_I_GAIN      = 82;
static constexpr uint16_t ADDR_POSITION_P_GAIN      = 84;
static constexpr uint16_t ADDR_PROFILE_ACCELERATION = 108;
static constexpr uint16_t ADDR_PROFILE_VELOCITY     = 112;
static constexpr uint16_t ADDR_GOAL_POSITION        = 116;
static constexpr uint16_t ADDR_PRESENT_CURRENT      = 126;
static constexpr uint16_t ADDR_PRESENT_POSITION     = 132;

// ==============================
// Position PID gain values written at torque-enable
// (was inline in enableTorque(): D=40, I=1, P=800)
// ==============================

static constexpr uint16_t ADDR_POSITION_P_GAIN_VALUE = 800;
static constexpr uint16_t ADDR_POSITION_I_GAIN_VALUE = 1;
static constexpr uint16_t ADDR_POSITION_D_GAIN_VALUE = 40;

// ==============================
// Motion profile parameters
// ==============================

static constexpr uint32_t OPERATING_VELOCITY    = 100;  // ~22.9 RPM (~137 deg/s), post-startup
static constexpr uint32_t PROFILE_ACCELERATION  = 40;   // ~8,583 rev/min², smooth accel/decel

// Slow startup: limit servo speed until first command + ramp duration.
// Profile Velocity unit = 0.229 RPM. Value 20 ≈ 4.6 RPM → ~2s for 45° move.
static constexpr uint32_t STARTUP_VELOCITY = 20;
static constexpr int      STARTUP_TICKS    = 50;  // 5 seconds at 10 Hz (accounts for up to ~130° move)

// ==============================
// Unit conversions
// ==============================

// 2048 ticks = pi rad (half of the 4096-tick/rev MX encoder).
static constexpr double DXL_RESOLUTION_HALF = 2048.0;

// Present-current LSB to milliamps.
static constexpr float CURRENT_TO_MA = 2.69f;

static constexpr double RAD_TO_DXL(double x) { return x / M_PI * DXL_RESOLUTION_HALF; }
static inline double DXL_TO_RAD(int32_t x) { return static_cast<double>(x) / DXL_RESOLUTION_HALF * M_PI; }

#endif  // ALBC_CONTROL_DYNAMIXEL_CONFIG_H
