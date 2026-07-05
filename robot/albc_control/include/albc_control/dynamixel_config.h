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
//
// Joint2 (ID 12) physical pose <-> commanded angle (measured 2026-07-05, user):
//   - This is a CONTINUOUS-ROTATION joint (no hard limit), so the angle is
//     absolute and wraps; the pose is what the angle means physically.
//   - theta2 = pi (~3.14 rad) => arm FULLY FOLDED (the reference/origin pose).
//   - theta2 = 0 (== 2*pi)    => arm FULLY EXTENDED (straight out); the angle
//                                wraps, so 2*pi is the same pose as 0 = extended.
//   - Fold amount tracks |theta2 - pi|: it folds monotonically toward pi, and
//     past pi (e.g. the 4.25 rad ~= 1.35*pi field reading) it unfolds again.
//   REFERENCE POSE for tank bring-up / B1 thruster probe: J1 = 0.0, J2 = pi.
//   Hold the arm here (torque on) so a slack/extended arm does not tip the
//   robot and confound thruster observations.
//   NOTE: supersedes the kinematic guess in the 2026-06-24 joint1-drift note
//   ("theta2=pi would give manip=0") -- the user's direct physical read wins.

#ifndef ALBC_CONTROL_DYNAMIXEL_CONFIG_H
#define ALBC_CONTROL_DYNAMIXEL_CONFIG_H

#include <cmath>     // M_PI
#include <cstdint>   // fixed-width integer types

// ==============================
// Bus / device configuration
// ==============================

static constexpr uint8_t  JOINT1_ID   = 11;
static constexpr uint8_t  JOINT2_ID   = 12;
// 1 Mbps: 모터 EEPROM baud(addr 8)와 반드시 일치. 2026-07-03 ID11/12 EEPROM을
// 57600->1M로 상향(값 3). 57600에서는 50Hz 루프의 position+current read+goal write
// 트래픽이 빠듯해 J2 고전류 시 COMM_RX_CORRUPT(err=-3002)가 간헐 발생했다. 1M로
// 올려 패킷 타이밍 여유 확보(SDK 지원: 57600/115200/1M/2M/3M/4M). 모터 baud를 되돌리려면
// EEPROM addr 8에 1(=57600) write. 이 값과 모터 EEPROM이 어긋나면 통신 두절.
static constexpr int      BAUDRATE    = 1000000;
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
