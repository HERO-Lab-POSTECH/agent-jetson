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
// Everything else is byte-identical to the board original (verified @ 2026-06-07).
// ------------------------------------------------------------------------------
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"          // RL-DEPLOY
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "albc_control/dynamixel_config.h"

#include <cmath>

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

void setPosition(uint8_t id, int32_t position) {
    uint8_t error = 0;
    int result = packet_handler->write4ByteTxRx(port_handler, id, ADDR_GOAL_POSITION, (unsigned int)position, &error);
    if (result != COMM_SUCCESS) {
        ROS_ERROR_THROTTLE(1.0, "Failed to set position %d for Dynamixel ID %d (err=%d)", position, id, result);
    }
}

void setProfileVelocity(uint8_t id, uint32_t velocity) {
    uint8_t error = 0;
    int result = packet_handler->write4ByteTxRx(port_handler, id, ADDR_PROFILE_VELOCITY, velocity, &error);
    if (result != COMM_SUCCESS) {
        ROS_WARN_THROTTLE(1.0, "Failed to set profile velocity %u for Dynamixel ID %d (err=%d)", velocity, id, result);
    }
}

int16_t readCurrent(uint8_t id) {
    uint8_t error = 0;
    int16_t current = 0;
    int result = packet_handler->read2ByteTxRx(port_handler, id, ADDR_PRESENT_CURRENT, (uint16_t*)&current, &error);
    if (result != COMM_SUCCESS) {
        ROS_ERROR_THROTTLE(1.0, "Failed to read current for Dynamixel ID %d (err=%d)", id, result);
    }
    return current;
}

int32_t readPosition(uint8_t id) {
    uint8_t error = 0;
    uint32_t raw = 0;
    int result = packet_handler->read4ByteTxRx(port_handler, id, ADDR_PRESENT_POSITION, &raw, &error);
    if (result != COMM_SUCCESS) {
        ROS_ERROR_THROTTLE(1.0, "Failed to read position for Dynamixel ID %d (err=%d)", id, result);
    }
    return static_cast<int32_t>(raw);
}

// RL-DEPLOY: read measured position, unwrap-accumulate, and differentiate (dt = loop period).
// Mirrors the sim's raw-cumulative joint_pos and gives a clean joint_vel at the true rate.
void updateMeasured(MeasuredState& m, uint8_t id, double dt) {
    double wrapped = DXL_TO_RAD(readPosition(id));      // raw reading, may wrap at +-pi/2pi
    // normalize to [0, 2pi) for a stable wrap comparison
    double w = fmod(wrapped, 2.0 * M_PI);
    if (w < 0.0) w += 2.0 * M_PI;
    if (!m.init) {
        m.prev_meas_wrapped = w;
        m.abs_meas = wrapped;     // seed cumulative with the raw reading
        m.vel = 0.0;
        m.init = true;
        return;
    }
    double delta = w - m.prev_meas_wrapped;
    if (delta > M_PI) delta -= 2.0 * M_PI;             // unwrap, same rule as updateJoint()
    else if (delta <= -M_PI) delta += 2.0 * M_PI;
    m.abs_meas += delta;
    m.vel = delta / dt;
    m.prev_meas_wrapped = w;
}

// ==============================
// Callback (DRY: handles both joints)
// ==============================

void updateJoint(JointState& joint, double commanded_angle) {
    double delta = commanded_angle - joint.prev_commanded;

    // Unwrap angle (handle 2pi wraparound)
    if (delta > M_PI) delta -= 2 * M_PI;
    else if (delta <= -M_PI) delta += 2 * M_PI;

    joint.absolute_angle += delta;
    joint.prev_commanded = commanded_angle;

    setPosition(joint.dxl_id, static_cast<int32_t>(RAD_TO_DXL(joint.absolute_angle)));
}

void joint1Callback(const std_msgs::Float64::ConstPtr& msg) {
    if (!first_command_received) first_command_received = true;
    updateJoint(joint1, msg->data);
}

void joint2Callback(const std_msgs::Float64::ConstPtr& msg) {
    if (!first_command_received) first_command_received = true;
    updateJoint(joint2, msg->data);
}

// ==============================
// Main
// ==============================

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_angle_command");
    ros::NodeHandle nh;

    ros::Publisher current_pub = nh.advertise<std_msgs::Float32MultiArray>("joint_currents", 10);
    // RL-DEPLOY: arm state for the RL inference node (pos = measured, vel = differentiated here)
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/albc/joint_states", 10);

    ros::Subscriber joint1_sub = nh.subscribe<std_msgs::Float64>(
        "/hero_agent/active_joint1_position_controller/command", 10, joint1Callback);
    ros::Subscriber joint2_sub = nh.subscribe<std_msgs::Float64>(
        "/hero_agent/active_joint2_position_controller/command", 10, joint2Callback);

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

    // Read current motor positions to initialize joint states (shortest-path rotation)
    int32_t pos1 = readPosition(JOINT1_ID);
    int32_t pos2 = readPosition(JOINT2_ID);
    double angle1 = DXL_TO_RAD(pos1);
    double angle2 = DXL_TO_RAD(pos2);

    joint1.absolute_angle = angle1;
    joint1.prev_commanded = fmod(angle1, 2.0 * M_PI);
    if (joint1.prev_commanded < 0.0) joint1.prev_commanded += 2.0 * M_PI;

    joint2.absolute_angle = angle2;
    joint2.prev_commanded = fmod(angle2, 2.0 * M_PI);
    if (joint2.prev_commanded < 0.0) joint2.prev_commanded += 2.0 * M_PI;

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
    ROS_INFO("  Startup: slow move for %.1f sec after first cmd", STARTUP_TICKS / 10.0);
    ROS_INFO("  RL-DEPLOY: publishing /albc/joint_states (measured pos + vel)");
    ROS_INFO("===================================");

    const double LOOP_HZ = 10.0;             // RL-DEPLOY: dt for velocity differentiation
    ros::Rate loop_rate(LOOP_HZ);
    const double dt = 1.0 / LOOP_HZ;

    while (ros::ok()) {
        // Restore full speed after startup ramp completes (timer starts on first command)
        if (first_command_received) {
            if (startup_counter == STARTUP_TICKS) {
                setProfileVelocity(JOINT1_ID, OPERATING_VELOCITY);
                setProfileVelocity(JOINT2_ID, OPERATING_VELOCITY);
                ROS_INFO("Startup ramp complete — operating velocity enabled (vel=%d, acc=%d)",
                         OPERATING_VELOCITY, PROFILE_ACCELERATION);
                startup_counter++;  // only run once
            } else if (startup_counter < STARTUP_TICKS) {
                startup_counter++;
            }
        }
        float current1_mA = static_cast<float>(readCurrent(JOINT1_ID)) * CURRENT_TO_MA;
        float current2_mA = static_cast<float>(readCurrent(JOINT2_ID)) * CURRENT_TO_MA;

        std_msgs::Float32MultiArray current_msg;
        current_msg.data = {current1_mA, current2_mA};
        current_pub.publish(current_msg);

        // RL-DEPLOY: read measured positions, accumulate + differentiate, publish JointState.
        updateMeasured(meas1, JOINT1_ID, dt);
        updateMeasured(meas2, JOINT2_ID, dt);
        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        js.name = {"albc_joint1", "albc_joint2"};
        js.position = {meas1.abs_meas, meas2.abs_meas};   // measured, raw-cumulative (matches sim)
        js.velocity = {meas1.vel, meas2.vel};             // differentiated at true 10 Hz
        joint_state_pub.publish(js);

        // [BUG FIX T1] Throttled logging (was unthrottled at 10 Hz)
        ROS_INFO_THROTTLE(2.0, "Joint Currents - J1: %.1f mA, J2: %.1f mA", current1_mA, current2_mA);

        ros::spinOnce();
        loop_rate.sleep();
    }

    port_handler->closePort();
    return 0;
}
