#ifndef HERO_AGENT_TYPES_H
#define HERO_AGENT_TYPES_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "hero_msgs/hero_agent_dvl.h"
#include "hero_msgs/hero_agent_sensor.h"

#include <cmath>
#include <map>
#include <functional>
#include <queue>

// ==============================
// Utility functions
// ==============================

template <typename T>
inline T clamp(T value, T min_val, T max_val) {
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

// ==============================
// State structures
// ==============================

struct TargetState {
    float x = 0, y = 0, z = 0, yaw = 0;
};

// ==============================
// Shared state (extern declarations)
// ==============================

extern TargetState target;

// ROS publishers
extern ros::Publisher pub_command;
extern ros::Publisher pub_target;
extern ros::Publisher pub_key_input;

// ROS messages
extern std_msgs::Int8 command_msg;
extern hero_msgs::hero_agent_dvl msg_target;

// Parameters loaded from YAML
extern double param_log_period;

// Teleop
extern double teleop_xy_step, teleop_z_step;
extern std::queue<int> key_input_queue;

// ==============================
// Module function declarations
// ==============================

// teleop.cpp
void init_keyboard();
void close_keyboard();
void handleKeyboardInput(ros::Rate& loop_rate);

#endif // HERO_AGENT_TYPES_H
