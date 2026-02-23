#ifndef HERO_AGENT_TYPES_H
#define HERO_AGENT_TYPES_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "hero_msgs/hero_agent_dvl.h"
#include "hero_msgs/hero_agent_cont_xy.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/ObjectCount.h"
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

struct DarknetState {
    int found = 0;
    int x = 0, y = 0, width = 0, height = 0, area = 0;
    float x_target = 320.0f, y_target = 300.0f;
    float x_error = 0, y_error = 0;
    float x_error_pre = 0, y_error_pre = 0;
    float x_error_d = 0, y_error_d = 0;
    float Kp = 0.1f, Kd = 0.1f;
    double saturation = 100.0;
};

struct LawnmowerState {
    int sway_count = 0, sway_num = 300;
    int surge_count = 0, surge_num = 50;
    float move_dis = 0.01f;
};

struct ControlFlags {
    int darknet = 0;
    int lawnmower = 0;
};

struct ThrustOutput {
    double Tx = 0, Ty = 0;
};

// ==============================
// Shared state (extern declarations)
// ==============================

extern TargetState target;
extern DarknetState darknet;
extern LawnmowerState lawnmower;
extern ControlFlags ctrl;
extern ThrustOutput thrust;

// ROS publishers
extern ros::Publisher pub_command;
extern ros::Publisher pub_target;
extern ros::Publisher pub_cont_xy;
extern ros::Publisher pub_key_input;

// ROS messages
extern std_msgs::Int8 command_msg;
extern hero_msgs::hero_agent_dvl msg_target;
extern hero_msgs::hero_agent_cont_xy cont_xy_msg;

// Parameters loaded from YAML
extern double param_log_period;

// Teleop
extern double teleop_xy_step, teleop_z_step;
extern std::queue<int> key_input_queue;

// ==============================
// Module function declarations
// ==============================

// darknet_control.cpp
void msgCallback_darknet(const darknet_ros_msgs::ObjectCount::ConstPtr &msg);
void msgCallback_darknet_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
void executeDarknetControl(int count);

// lawnmower_survey.cpp
void executeLawnmowerSurvey(int count);

// teleop.cpp
void init_keyboard();
void close_keyboard();
void handleKeyboardInput(ros::Rate& loop_rate);

#endif // HERO_AGENT_TYPES_H
