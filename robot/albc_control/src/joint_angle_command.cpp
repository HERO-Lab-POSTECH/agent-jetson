// joint_angle_command.cpp  —  PATCHED for RL deployment
// ------------------------------------------------------------------------------
// Adds a /albc/joint_states publisher to the existing Dynamixel driver so the RL
// inference node can read the arm state WITHOUT opening the serial port itself
// (the port has a single owner -- this node). Two deliberate choices, both decided
// against the sim contract (observations.py robot.data.joint_pos / joint_vel):
//
//   1. joint POSITION published = ACTUAL measured position (readPosition every loop,
//      unwrapped+accumulated the same way absolute_angle is), NOT the commanded
//      absolute_angle. The sim obs uses robot.data.joint_pos (true state), so the
//      deployed obs must use the measured encoder value to match.
//
//   2. joint VELOCITY published = differentiated HERE at the true loop rate (10 Hz),
//      shipped in JointState.velocity. The RL node runs at 50 Hz; if it re-diff'd a
//      10 Hz position it would see a 4-tick-zero / 1-tick-spike staircase. The driver
//      knows its real dt, so it owns the derivative.
//
// DIFF FROM ORIGINAL (search "RL-DEPLOY"):
//   + #include <sensor_msgs/JointState.h>
//   + readPositionRad() helper + per-joint measured-angle unwrap accumulator
//   + joint_state_pub + publishJointStates() in the main loop
//   + (2026-06-12 field-test audit fixes)
//     - command subscribers queue_size 10 -> 1 (50 Hz cmd vs 10 Hz loop: stale-burst
//       of ~5 serial writes per joint per cycle -> exactly one, latest wins)
//     - readPosition() returns success; a failed read no longer injects raw=0 as a
//       real angle (+-pi fake delta, +-31 rad/s velocity spike into the RL obs).
//       Failed cycles skip the JointState publish; failed INIT reads are fatal.
//     - velocity differentiation uses the measured loop dt (ros::Time), not 1/LOOP_HZ
//     - Dynamixel hardware error byte surfaced (WARN_THROTTLE) on read/write
//     - one-shot "First command received" INFO so the operator sees cmd flow
//   + (2026-08-17 cable-break fix — see joint_unwrap.h for the derivation)
//     - unwrap is now NEAREST-equivalent, not single-step. The old rule leaked
//       (k-1) whole turns on the first command whenever the arm started more
//       than one turn from zero, which broke the J1->J2 cable on 2026-08-13.
//     - joint1 cable guard: ~joint1_abort_rad (default 6pi = 3 turns) ABORTS the
//       run — latches, stops applying commands, holds the last goal, leaves
//       torque ON. Checked on BOTH the commanded and the MEASURED angle.
//     - constraint metering on /albc/joint_guard. Counted, never enforced: a
//       silent clamp would destroy the quantity used to compare TDC / classic
//       PID / RL, and all four entry points drive the arm through this node.
// Everything else is byte-identical to the board original (verified @ 2026-06-07).
// ------------------------------------------------------------------------------
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"      // RL-DEPLOY 2026-08-17 (/albc/joint_guard)
#include "sensor_msgs/JointState.h"          // RL-DEPLOY
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "albc_control/dynamixel_config.h"
#include "albc_control/joint_unwrap.h"       // RL-DEPLOY 2026-08-17 (cable-break fix)

#include <cmath>

// RL-DEPLOY 2026-08-13: the driver's read loop rate, from ~loop_hz. It used to be
// a hardcoded 10.0 here while the RL node ran at 50 Hz, so the policy integrated its
// joint-target accumulator across 4-5 ticks of the SAME measurement. Equalising the
// per-second joint gain did NOT fix the resulting oscillation (control_hz 50 with
// joint_delta_scale 0.02 was still unstable while control_hz 10 was calm), which
// leaves the rate mismatch itself. Made a parameter, not a new hardcoded 50, because
// the right value is a measurement: the Dynamixel bus has to sustain it.
// Default 10.0 preserves the shipped behaviour when the param is unset.
// g_startup_ticks and g_read_fail_limit are DERIVED so the 5 s startup ramp and the
// ~1 s comms-outage threshold stay the same wall-clock durations at any loop rate.
static double g_loop_hz         = 10.0;
static int    g_startup_ticks   = STARTUP_TICKS;   // 50 = 5 s at 10 Hz
static int    g_read_fail_limit = 10;              // 10 = ~1 s at 10 Hz

// RL-DEPLOY 2026-08-17: joint1 cable guard + constraint metering.
// Three bands, and only the outermost one intervenes:
//   |theta1| <= 4pi   free. This is the TRAINING constraint band
//                     (joint1_position_cost, limit_rad 4pi, budget 0.01).
//   4pi .. 6pi        free, but METERED -- see g_j1_over_count.
//   |theta1| >  6pi   ABORT (operator decision 2026-08-17: 3 turns is the last
//                     line, 4 turns is dangerous).
//
// Metering, not clamping, is the point. A silent clamp rewrites the commanded
// value, which destroys the very quantity we want to compare across TDC,
// classic PID and RL -- how far the controller actually tried to go. And the
// old policy-side clamp is what turned a seed outside the band into a
// multi-radian step command. So: count inside the band, abort at the ceiling,
// never truncate.
//
// The guard watches the MEASURED angle as well as the commanded one, because
// on 2026-08-13 the command stream stayed inside +-3 turns (max |cmd| 18.755)
// while the arm was driven to -35.54. A command-side-only guard misses exactly
// the failure this exists to prevent.
static double g_j1_abort_rad   = 6.0 * M_PI;   // 3 turns -- cable ceiling, ABORTS
static double g_j1_count_rad   = 4.0 * M_PI;   // 2 turns -- training limit, COUNTS
static bool   g_abort_latched  = false;
static double g_j1_over_count  = 0.0;          // ticks with |measured theta1| > count_rad
static double g_j2_over_pi     = 0.0;          // ticks with |measured theta2| > pi
static double g_j1_abs_max     = 0.0;
static double g_j2_abs_max     = 0.0;

// ==============================
// Joint State
// ==============================

struct JointState {
    double prev_commanded;
    double absolute_angle;
    uint8_t dxl_id;
};

static JointState joint1 = {0.0, 0.0, JOINT1_ID};
static JointState joint2 = {0.0, 0.0, JOINT2_ID};

// RL-DEPLOY: measured-position accumulators (separate from commanded absolute_angle).
// We unwrap the raw encoder angle into a cumulative value to match the sim's
// raw-cumulative joint_pos, and differentiate it for velocity.
struct MeasuredState {
    double prev_meas_wrapped;   // last wrapped reading in [0,2pi)
    double abs_meas;            // unwrapped cumulative measured angle (rad)
    double vel;                 // differentiated velocity (rad/s)
    bool   init;
};
static MeasuredState meas1 = {0.0, 0.0, 0.0, false};
static MeasuredState meas2 = {0.0, 0.0, 0.0, false};

static dynamixel::PortHandler*   port_handler   = nullptr;
static dynamixel::PacketHandler* packet_handler = nullptr;

static bool first_command_received = false;
static int  startup_counter = 0;
// RL-DEPLOY (2026-06-12 logging): consecutive joint read-fail counter to distinguish
// a one-off serial glitch from a sustained motor-comms outage (power/relay/cable/HW).
static int  consecutive_read_fail = 0;

// ==============================
// Dynamixel Helpers
// ==============================

void enableTorque(uint8_t id) {
    uint8_t error = 0;
    int result;
    result = packet_handler->write1ByteTxRx(port_handler, id, ADDR_TORQUE_ENABLE, 1, &error);
    if (result != COMM_SUCCESS) {
        ROS_ERROR_THROTTLE(1.0, "Failed to enable torque for Dynamixel ID %d (err=%d)", id, result);
    }
    result = packet_handler->write2ByteTxRx(port_handler, id, ADDR_POSITION_D_GAIN, ADDR_POSITION_D_GAIN_VALUE, &error);
    if (result != COMM_SUCCESS) {
        ROS_WARN_THROTTLE(1.0, "Failed to set position D gain for Dynamixel ID %d (err=%d)", id, result);
    }
    result = packet_handler->write2ByteTxRx(port_handler, id, ADDR_POSITION_I_GAIN, ADDR_POSITION_I_GAIN_VALUE, &error);
    if (result != COMM_SUCCESS) {
        ROS_WARN_THROTTLE(1.0, "Failed to set position I gain for Dynamixel ID %d (err=%d)", id, result);
    }
    result = packet_handler->write2ByteTxRx(port_handler, id, ADDR_POSITION_P_GAIN, ADDR_POSITION_P_GAIN_VALUE, &error);
    if (result != COMM_SUCCESS) {
        ROS_WARN_THROTTLE(1.0, "Failed to set position P gain for Dynamixel ID %d (err=%d)", id, result);
    }
    result = packet_handler->write4ByteTxRx(port_handler, id, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &error);
    if (result != COMM_SUCCESS) {
        ROS_WARN_THROTTLE(1.0, "Failed to set profile acceleration for Dynamixel ID %d (err=%d)", id, result);
    }
}

// RL-DEPLOY: release torque so the arm free-wheels and can be repositioned by
// hand. Called on node shutdown -- previously torque stayed ON after the node
// exited, so the only way to free the joints was cutting motor power (relay OFF).
void disableTorque(uint8_t id) {
    uint8_t error = 0;
    int result = packet_handler->write1ByteTxRx(port_handler, id, ADDR_TORQUE_ENABLE, 0, &error);
    if (result != COMM_SUCCESS) {
        ROS_ERROR("Failed to disable torque for Dynamixel ID %d (err=%d)", id, result);
    }
}

void setPosition(uint8_t id, int32_t position) {
    uint8_t error = 0;
    int result = packet_handler->write4ByteTxRx(port_handler, id, ADDR_GOAL_POSITION, (unsigned int)position, &error);
    if (result != COMM_SUCCESS) {
        ROS_ERROR_THROTTLE(1.0, "Failed to set position %d for Dynamixel ID %d (err=%d)", position, id, result);
    } else if (error != 0) {
        // RL-DEPLOY: device-reported hardware error (overload/voltage/...) was silent
        ROS_WARN_THROTTLE(1.0, "Dynamixel ID %d hardware error byte 0x%02X on goal write", id, error);
    }
}

void setProfileVelocity(uint8_t id, uint32_t velocity) {
    uint8_t error = 0;
    int result = packet_handler->write4ByteTxRx(port_handler, id, ADDR_PROFILE_VELOCITY, velocity, &error);
    if (result != COMM_SUCCESS) {
        ROS_WARN_THROTTLE(1.0, "Failed to set profile velocity %u for Dynamixel ID %d (err=%d)", velocity, id, result);
    }
}

// RL-DEPLOY 2026-08-25: returns success instead of silently yielding raw=0, for the
// same reason readPosition below does. A failed read used to publish 0 mA, and the RL
// node's SUSTAINED over-current guard treats any under-cap sample as proof the stall
// cleared -- so one failed read per cur_max_s window reset the timer and the guard
// could never trip. This bus has measured 295/572 read failures under load
// (2026-08-20), and a loaded, noisy bus is exactly what a stall creates.
bool readCurrent(uint8_t id, int16_t* out) {
    uint8_t error = 0;
    int16_t current = 0;
    int result = packet_handler->read2ByteTxRx(port_handler, id, ADDR_PRESENT_CURRENT, (uint16_t*)&current, &error);
    if (result != COMM_SUCCESS) {
        ROS_ERROR_THROTTLE(1.0, "Failed to read current for Dynamixel ID %d (err=%d)", id, result);
        return false;
    }
    *out = current;
    return true;
}

// RL-DEPLOY: returns success instead of silently yielding raw=0 on a failed read.
// A comm glitch used to inject 0 rad as a real measurement: up to +-pi of fake
// delta and a +-31 rad/s velocity spike straight into the RL observation.
bool readPosition(uint8_t id, int32_t* out) {
    uint8_t error = 0;
    uint32_t raw = 0;
    int result = packet_handler->read4ByteTxRx(port_handler, id, ADDR_PRESENT_POSITION, &raw, &error);
    if (result != COMM_SUCCESS) {
        ROS_ERROR_THROTTLE(1.0, "Failed to read position for Dynamixel ID %d (err=%d)", id, result);
        return false;
    }
    if (error != 0) {
        ROS_WARN_THROTTLE(1.0, "Dynamixel ID %d hardware error byte 0x%02X on position read", id, error);
    }
    *out = static_cast<int32_t>(raw);
    return true;
}

// RL-DEPLOY: read measured position, unwrap-accumulate, and differentiate (dt = TRUE
// loop period, measured by the caller). Mirrors the sim's raw-cumulative joint_pos.
// Returns false when the serial read failed -- the caller must skip publishing this
// cycle so a glitch never corrupts the RL observation.
bool updateMeasured(MeasuredState& m, uint8_t id, double dt) {
    int32_t raw_pos = 0;
    if (!readPosition(id, &raw_pos)) return false;
    double wrapped = DXL_TO_RAD(raw_pos);               // raw reading, may wrap at +-pi/2pi
    // normalize to [0, 2pi) for a stable wrap comparison
    double w = fmod(wrapped, 2.0 * M_PI);
    if (w < 0.0) w += 2.0 * M_PI;
    if (!m.init) {
        m.prev_meas_wrapped = w;
        m.abs_meas = wrapped;     // seed cumulative with the raw reading
        m.vel = 0.0;
        m.init = true;
        return true;
    }
    // same rule as updateJoint(). Consecutive readings are always well under pi
    // apart (OPERATING_VELOCITY caps the joint near 2.4 rad/s, so <=0.24 rad per
    // 10 Hz tick), which is the region where nearest and single-step agree.
    double delta = albc::unwrapNearest(w - m.prev_meas_wrapped);
    m.abs_meas += delta;
    m.vel = delta / dt;
    m.prev_meas_wrapped = w;
    return true;
}

// ==============================
// Callback (DRY: handles both joints)
// ==============================

// RL-DEPLOY 2026-08-17: latch the cable guard. Deliberately does NOT disable
// torque -- underwater a limp arm falls, which is worse than a held one. The arm
// keeps its last written goal, commands stop being applied, and the operator
// decides. Unwinding J1 is a manual step before the next run.
void tripAbort(const char* what, double value) {
    if (g_abort_latched) return;
    g_abort_latched = true;
    ROS_ERROR("JOINT GUARD TRIPPED: %s = %.3f rad (%.2f turns) exceeds the "
              "%.2f-turn cable ceiling (~joint1_abort_rad). Commands are now "
              "IGNORED and the arm HOLDS its last goal; torque stays ON. "
              "Unwind J1 before restarting.",
              what, value, value / (2.0 * M_PI), g_j1_abort_rad / (2.0 * M_PI));
}

void updateJoint(JointState& joint, double commanded_angle) {
    // Guard latched: stop following the topic. Holding the last goal is the
    // abort behaviour -- see tripAbort().
    if (g_abort_latched) return;

    double raw_delta = commanded_angle - joint.prev_commanded;

    // Unwrap to the NEAREST equivalent angle, removing however many whole turns
    // separate the command from the running baseline. The former rule removed at
    // most one 2pi, so a cumulative-vs-wrapped baseline mismatch of k turns
    // leaked (k-1) turns into absolute_angle on the first command -- the
    // 2026-08-13 cable break. Identical to the old rule for |delta| < pi, which
    // is every steady-state tick. See joint_unwrap.h for the full derivation.
    double delta = albc::unwrapNearest(raw_delta);

    // Say so when a fold actually happens. A per-tick request more than pi away
    // from the baseline is never legitimate here (the RL accumulator moves at
    // most DELTA_SCALE = 0.10 rad/tick; the classic publisher is continuous), so
    // it means either a representation mismatch or a truncated policy seed.
    // Folding it silently is what kept the old bug invisible.
    if (std::fabs(raw_delta - delta) > 1e-9) {
        ROS_WARN_THROTTLE(1.0,
            "joint ID %d: command %.3f rad is %.2f turns from baseline %.3f -- "
            "folded to a %.3f rad step. Representation mismatch or clipped seed.",
            joint.dxl_id, commanded_angle,
            (raw_delta - delta) / (2.0 * M_PI), joint.prev_commanded, delta);
    }

    joint.absolute_angle += delta;
    joint.prev_commanded = commanded_angle;

    // Command-side ceiling: catches intent one tick early. The measured-side
    // check in the main loop is the one that catches a driver/arm divergence.
    if (joint.dxl_id == JOINT1_ID &&
        albc::overGuard(joint.absolute_angle, g_j1_abort_rad)) {
        tripAbort("joint1 commanded", joint.absolute_angle);
        return;                    // never write the offending goal
    }

    setPosition(joint.dxl_id, static_cast<int32_t>(RAD_TO_DXL(joint.absolute_angle)));
}

// RL-DEPLOY: one-shot INFO so the operator can SEE the first command arrive
// (previously nothing was printed until the ramp-complete message ~5 s later).
void noteFirstCommand(int joint_no, double angle) {
    if (!first_command_received) {
        first_command_received = true;
        ROS_INFO("First command received (joint%d = %.3f rad) -- startup ramp begins (%.1f s)",
                 joint_no, angle, g_startup_ticks / g_loop_hz);
    }
}

void joint1Callback(const std_msgs::Float64::ConstPtr& msg) {
    noteFirstCommand(1, msg->data);
    updateJoint(joint1, msg->data);
}

void joint2Callback(const std_msgs::Float64::ConstPtr& msg) {
    noteFirstCommand(2, msg->data);
    updateJoint(joint2, msg->data);
}

// ==============================
// Main
// ==============================

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_angle_command");
    ros::NodeHandle nh;

    ros::Publisher current_pub = nh.advertise<std_msgs::Float32MultiArray>("/joint_currents", 10);
    // RL-DEPLOY: arm state for the RL inference node (pos = measured, vel = differentiated here)
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/albc/joint_states", 10);
    // RL-DEPLOY 2026-08-17: constraint metering + guard state. Every controller
    // (TDC, classic PID, RL, B1 probe) drives the arm through THIS node, so a
    // counter here measures all of them on one instrument and they stay
    // comparable. Counting per-controller would give each its own definition.
    //
    // CONTRACT -- 5 fields, fixed order, never reorder (consumers index by position):
    //   0 j1_over_count : ticks with |measured theta1| > ~joint1_count_rad (4pi)
    //                     == the training constraint joint1_position_cost
    //   1 j1_abs_max    : max |measured theta1| this run (rad)
    //   2 j2_over_pi    : ticks with |measured theta2| > pi (manipulability
    //                     singularity side; metered only, never enforced)
    //   3 j2_abs_max    : max |measured theta2| this run (rad)
    //   4 abort_flag    : 1.0 once the cable guard has latched, else 0.0
    ros::Publisher guard_pub = nh.advertise<std_msgs::Float64MultiArray>("/albc/joint_guard", 10);

    // RL-DEPLOY: queue_size 1 -- the RL node publishes at 50 Hz but this loop drains
    // callbacks at 10 Hz; with queue 10 every cycle replayed ~5 stale commands per
    // joint, each one a serial goal write. Keeping only the LATEST command caps the
    // serial traffic at one goal write per joint per cycle.
    ros::Subscriber joint1_sub = nh.subscribe<std_msgs::Float64>(
        "/hero_agent/active_joint1_position_controller/command", 1, joint1Callback);
    ros::Subscriber joint2_sub = nh.subscribe<std_msgs::Float64>(
        "/hero_agent/active_joint2_position_controller/command", 1, joint2Callback);

    port_handler   = dynamixel::PortHandler::getPortHandler(SERIAL_PORT);
    packet_handler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL);

    if (!port_handler->openPort()) {
        ROS_ERROR("Failed to open port: %s", SERIAL_PORT);
        return -1;
    }

    if (!port_handler->setBaudRate(BAUDRATE)) {
        ROS_ERROR("Failed to set baudrate: %d", BAUDRATE);
        return -1;
    }

    enableTorque(JOINT1_ID);
    enableTorque(JOINT2_ID);

    // Read current motor positions to initialize joint states (shortest-path rotation).
    // RL-DEPLOY: a failed init read used to seed prev_commanded/absolute_angle with 0,
    // silently corrupting the command baseline -- refuse to start instead.
    int32_t pos1 = 0, pos2 = 0;
    if (!readPosition(JOINT1_ID, &pos1) || !readPosition(JOINT2_ID, &pos2)) {
        ROS_ERROR("Initial position read failed -- refusing to start with a corrupt baseline");
        port_handler->closePort();
        return -1;
    }
    double angle1 = DXL_TO_RAD(pos1);
    double angle2 = DXL_TO_RAD(pos2);

    // The two halves are in DIFFERENT representations of the same angle:
    // absolute_angle is cumulative (multi-turn, operating mode 4), prev_commanded
    // is wrapped into [0, 2pi). That is deliberate -- this topic has publishers in
    // BOTH conventions (rl_inference_node sends cumulative, status_publisher.h
    // sends mapTo2Pi wrapped) and a wrapped baseline is the one that suits both.
    //
    // It was only ever safe once updateJoint started unwrapping to the NEAREST
    // equivalent (2026-08-17). Under the old single-step unwrap this pairing
    // leaked (k-1) whole turns into absolute_angle on the first cumulative
    // command and broke the J1->J2 cable on 2026-08-13. Do not "fix" this to a
    // cumulative baseline: that merely moves the same defect onto the classic
    // publisher. joint_unwrap.h carries the derivation.
    joint1.absolute_angle = angle1;
    joint1.prev_commanded = albc::wrapTo2Pi(angle1);

    joint2.absolute_angle = angle2;
    joint2.prev_commanded = albc::wrapTo2Pi(angle2);

    // Slow startup: limit servo speed until first command + ramp duration
    // (STARTUP_VELOCITY / STARTUP_TICKS defined in dynamixel_config.h)
    setProfileVelocity(JOINT1_ID, STARTUP_VELOCITY);
    setProfileVelocity(JOINT2_ID, STARTUP_VELOCITY);

    ROS_INFO("===================================");
    ROS_INFO("  Joint Angle Command Initialized");
    ROS_INFO("  Port: %s  Baud: %d", SERIAL_PORT, BAUDRATE);
    ROS_INFO("  Joint1 ID=%d  Joint2 ID=%d", JOINT1_ID, JOINT2_ID);
    ROS_INFO("  J1 present: %.1f deg  J2 present: %.1f deg",
             angle1 * 180.0 / M_PI, angle2 * 180.0 / M_PI);
    ROS_INFO("  Startup: slow move for %.1f sec after first cmd",
             g_startup_ticks / g_loop_hz);
    ROS_INFO("  RL-DEPLOY: publishing /albc/joint_states (measured pos + vel)");
    ROS_INFO("===================================");

    // ~loop_hz: how fast this driver reads the servos and republishes
    // /albc/joint_states. The RL node's control_hz must not outrun it -- see the
    // g_loop_hz comment at the top of this file. Clamped to a sane band so a typo
    // cannot flood the 1 Mbps Dynamixel bus or stall the loop to a crawl.
    ros::NodeHandle pnh("~");
    pnh.param("loop_hz", g_loop_hz, 10.0);
    if (g_loop_hz < 1.0 || g_loop_hz > 100.0) {
        ROS_WARN("loop_hz %.1f out of [1, 100] -- clamping", g_loop_hz);
        g_loop_hz = std::min(100.0, std::max(1.0, g_loop_hz));
    }
    // keep the wall-clock durations these counters were tuned for
    g_startup_ticks   = static_cast<int>(STARTUP_TICKS / 10.0 * g_loop_hz + 0.5);
    g_read_fail_limit = static_cast<int>(1.0 * g_loop_hz + 0.5);
    ROS_INFO("  loop_hz: %.1f Hz  (startup ramp %d ticks = %.1f s, "
             "read-fail alarm %d ticks = %.1f s)",
             g_loop_hz, g_startup_ticks, g_startup_ticks / g_loop_hz,
             g_read_fail_limit, g_read_fail_limit / g_loop_hz);

    // RL-DEPLOY 2026-08-17: cable guard band. Defaults are the operator's
    // 2026-08-17 decision (3 turns is the last line, 4 turns is dangerous) and
    // the training constraint (joint1_position_cost limit_rad = 4pi). Set
    // ~joint1_abort_rad <= 0 to disable the abort entirely -- metering stays on.
    pnh.param("joint1_abort_rad", g_j1_abort_rad, 6.0 * M_PI);
    pnh.param("joint1_count_rad", g_j1_count_rad, 4.0 * M_PI);
    ROS_INFO("  joint1 guard: count > %.2f turns, ABORT > %.2f turns%s",
             g_j1_count_rad / (2.0 * M_PI), g_j1_abort_rad / (2.0 * M_PI),
             g_j1_abort_rad > 0.0 ? "" : "  (abort DISABLED)");

    ros::Rate loop_rate(g_loop_hz);
    // RL-DEPLOY: differentiate with the MEASURED loop period, not 1/LOOP_HZ -- when
    // the loop runs long (serial latency) a fixed dt systematically overestimates
    // the published joint velocity, which feeds the RL observation.
    ros::Time prev_loop_t = ros::Time::now();

    while (ros::ok()) {
        // Restore full speed after startup ramp completes (timer starts on first command)
        if (first_command_received) {
            if (startup_counter == g_startup_ticks) {
                setProfileVelocity(JOINT1_ID, OPERATING_VELOCITY);
                setProfileVelocity(JOINT2_ID, OPERATING_VELOCITY);
                ROS_INFO("Startup ramp complete — operating velocity enabled (vel=%d, acc=%d)",
                         OPERATING_VELOCITY, PROFILE_ACCELERATION);
                startup_counter++;  // only run once
            } else if (startup_counter < g_startup_ticks) {
                startup_counter++;
            }
        }
        // Publish ONLY when both reads succeeded. A half-read pair would put a real
        // current next to a fabricated 0, and the consumer takes max(|.|) over both.
        int16_t raw1 = 0, raw2 = 0;
        bool cur_ok = readCurrent(JOINT1_ID, &raw1) && readCurrent(JOINT2_ID, &raw2);
        float current1_mA = static_cast<float>(raw1) * CURRENT_TO_MA;
        float current2_mA = static_cast<float>(raw2) * CURRENT_TO_MA;
        if (cur_ok) {
            std_msgs::Float32MultiArray current_msg;
            current_msg.data = {current1_mA, current2_mA};
            current_pub.publish(current_msg);
        }

        // RL-DEPLOY: read measured positions, accumulate + differentiate, publish JointState.
        ros::Time now_t = ros::Time::now();
        double dt = (now_t - prev_loop_t).toSec();
        prev_loop_t = now_t;
        if (dt < 0.005) dt = 0.005;          // clamp against timer glitches; the init
        if (dt > 1.0)   dt = 1.0;            // path ignores dt entirely
        bool ok1 = updateMeasured(meas1, JOINT1_ID, dt);
        bool ok2 = updateMeasured(meas2, JOINT2_ID, dt);
        if (ok1 && ok2) {
            consecutive_read_fail = 0;   // RL-DEPLOY: a good read clears the outage counter
            sensor_msgs::JointState js;
            js.header.stamp = now_t;
            js.name = {"albc_joint1", "albc_joint2"};
            js.position = {meas1.abs_meas, meas2.abs_meas};   // measured, raw-cumulative (matches sim)
            js.velocity = {meas1.vel, meas2.vel};             // differentiated at the true rate
            joint_state_pub.publish(js);

            // RL-DEPLOY 2026-08-17: meter on the MEASURED angle, and guard on it.
            // This is the check that would have caught 2026-08-13: the command
            // stream never left +-3 turns while the arm was driven to -35.54 rad,
            // so watching only what was commanded misses a driver/arm divergence.
            const double a1 = std::fabs(meas1.abs_meas);
            const double a2 = std::fabs(meas2.abs_meas);
            if (a1 > g_j1_abs_max) g_j1_abs_max = a1;
            if (a2 > g_j2_abs_max) g_j2_abs_max = a2;
            if (a1 > g_j1_count_rad) g_j1_over_count += 1.0;   // counted, NOT enforced
            if (a2 > M_PI)           g_j2_over_pi    += 1.0;   // counted, NOT enforced
            if (albc::overGuard(meas1.abs_meas, g_j1_abort_rad)) {
                tripAbort("joint1 measured", meas1.abs_meas);
            }
        } else {
            // skip the publish: the RL node's staleness gate handles a long outage,
            // and a single glitch must not reach the observation as fake state.
            ROS_WARN_THROTTLE(1.0, "joint position read failed -- joint_states publish skipped");
            // RL-DEPLOY (2026-06-12 logging): surface a SUSTAINED outage distinctly from a glitch.
            ++consecutive_read_fail;
            if (consecutive_read_fail >= g_read_fail_limit) {  // ~1 s at any loop_hz
                ROS_ERROR_THROTTLE(1.0,
                    "joint read FAILED %d cycles in a row -- motor comms likely DOWN "
                    "(power/relay off? serial cable? Dynamixel HW error?)", consecutive_read_fail);
            }
        }

        // RL-DEPLOY 2026-08-17: guard/metering state, every cycle (5 fields, see
        // the guard_pub advertise above for the field contract).
        std_msgs::Float64MultiArray guard_msg;
        guard_msg.data = {g_j1_over_count, g_j1_abs_max,
                          g_j2_over_pi,    g_j2_abs_max,
                          g_abort_latched ? 1.0 : 0.0};
        guard_pub.publish(guard_msg);

        // [BUG FIX T1] Throttled logging (was unthrottled at 10 Hz)
        ROS_INFO_THROTTLE(2.0, "Joint Currents - J1: %.1f mA, J2: %.1f mA%s",
                          current1_mA, current2_mA,
                          cur_ok ? "" : "   *** READ FAILED, not published ***");
        ROS_INFO_THROTTLE(5.0,
            "joint1 |theta| max %.2f turns, %d ticks past the %.1f-turn training "
            "limit%s", g_j1_abs_max / (2.0 * M_PI), static_cast<int>(g_j1_over_count),
            g_j1_count_rad / (2.0 * M_PI), g_abort_latched ? "  [GUARD LATCHED]" : "");

        ros::spinOnce();
        loop_rate.sleep();
    }

    // RL-DEPLOY: release joint torque on shutdown so the arm can be moved by
    // hand once the node exits. Runs on graceful (SIGINT) shutdown, before the
    // port closes. WARNING: if the arm is holding a pose out of water, it will
    // drop under gravity when torque releases.
    disableTorque(JOINT1_ID);
    disableTorque(JOINT2_ID);
    ROS_INFO("Joint torque released -- arm is now free to move by hand.");

    port_handler->closePort();
    return 0;
}
