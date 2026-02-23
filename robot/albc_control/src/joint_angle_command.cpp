#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <cmath>

// Dynamixel configuration
static constexpr uint8_t  JOINT1_ID   = 11;
static constexpr uint8_t  JOINT2_ID   = 12;
static constexpr int      BAUDRATE    = 57600;
static const char*        SERIAL_PORT = "/dev/ttyDynamixel";
static constexpr float    PROTOCOL    = 2.0;

// Dynamixel register addresses
static constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
static constexpr uint16_t ADDR_POSITION_D_GAIN  = 80;
static constexpr uint16_t ADDR_POSITION_I_GAIN  = 82;
static constexpr uint16_t ADDR_POSITION_P_GAIN  = 84;
static constexpr uint16_t ADDR_GOAL_POSITION    = 116;
static constexpr uint16_t ADDR_PRESENT_CURRENT  = 126;

static constexpr double RAD_TO_DXL(double x) { return x / M_PI * 2048.0; }

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

static dynamixel::PortHandler*   port_handler   = nullptr;
static dynamixel::PacketHandler* packet_handler = nullptr;

// ==============================
// Dynamixel Helpers
// ==============================

void enableTorque(uint8_t id) {
    uint8_t error = 0;
    packet_handler->write1ByteTxRx(port_handler, id, ADDR_TORQUE_ENABLE, 1, &error);
    packet_handler->write2ByteTxRx(port_handler, id, ADDR_POSITION_D_GAIN, 40, &error);
    packet_handler->write2ByteTxRx(port_handler, id, ADDR_POSITION_I_GAIN, 1, &error);
    packet_handler->write2ByteTxRx(port_handler, id, ADDR_POSITION_P_GAIN, 800, &error);
}

void setPosition(uint8_t id, int32_t position) {
    uint8_t error = 0;
    int result = packet_handler->write4ByteTxRx(port_handler, id, ADDR_GOAL_POSITION, (unsigned int)position, &error);
    if (result != COMM_SUCCESS) {
        ROS_ERROR_THROTTLE(1.0, "Failed to set position %d for Dynamixel ID %d (err=%d)", position, id, result);
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
    updateJoint(joint1, msg->data);
}

void joint2Callback(const std_msgs::Float64::ConstPtr& msg) {
    updateJoint(joint2, msg->data);
}

// ==============================
// Main
// ==============================

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_angle_command");
    ros::NodeHandle nh;

    ros::Publisher current_pub = nh.advertise<std_msgs::Float32MultiArray>("joint_currents", 10);

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

    ROS_INFO("===================================");
    ROS_INFO("  Joint Angle Command Initialized");
    ROS_INFO("  Port: %s  Baud: %d", SERIAL_PORT, BAUDRATE);
    ROS_INFO("  Joint1 ID=%d  Joint2 ID=%d", JOINT1_ID, JOINT2_ID);
    ROS_INFO("===================================");

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        float current1_mA = static_cast<float>(readCurrent(JOINT1_ID)) * 2.69f;
        float current2_mA = static_cast<float>(readCurrent(JOINT2_ID)) * 2.69f;

        std_msgs::Float32MultiArray current_msg;
        current_msg.data = {current1_mA, current2_mA};
        current_pub.publish(current_msg);

        // [BUG FIX T1] Throttled logging (was unthrottled at 10 Hz)
        ROS_INFO_THROTTLE(2.0, "Joint Currents - J1: %.1f mA, J2: %.1f mA", current1_mA, current2_mA);

        ros::spinOnce();
        loop_rate.sleep();
    }

    port_handler->closePort();
    return 0;
}
