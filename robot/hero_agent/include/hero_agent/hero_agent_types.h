#ifndef HERO_AGENT_TYPES_H
#define HERO_AGENT_TYPES_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int64.h>
#include "hero_msgs/hero_agent_dvl.h"
#include "hero_msgs/hero_agent_position_result.h"
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

struct NavigationState {
    float x = 0, y = 0, z = 0, yaw = 0;
};

struct TargetState {
    float x = 0, y = 0, z = 0, yaw = 0;
};

struct WinchState {
    int64_t current_position = 0;
    int64_t target_position = 0;
    int64_t calib_position = 0;
    float target_meter = 0;
    float pre_target_meter = 0;
    float target_rotation = 0;
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

struct MosaicState {
    int sway_count = 0, sway_num = 300;
    int surge_count = 0, surge_num = 50;
    float move_dis = 0.01f;
    int deep_count = 0, start_position = 0;
};

struct ControlFlags {
    int darknet = 0;
    int mosaic = 0;
};

struct ThrustOutput {
    double Tx = 0, Ty = 0;
};

// ==============================
// Shared state (extern declarations)
// ==============================

extern NavigationState navi;
extern TargetState target;
extern WinchState winch;
extern DarknetState darknet;
extern MosaicState mosaic;
extern ControlFlags ctrl;
extern ThrustOutput thrust;

// ROS publishers
extern ros::Publisher pub_command;
extern ros::Publisher pub_target;
extern ros::Publisher pub_winch_target;
extern ros::Publisher pub_cont_xy;
extern ros::Publisher pub_key_input;

// ROS messages
extern std_msgs::Int8 command_msg;
extern hero_msgs::hero_agent_dvl msg_target;
extern std_msgs::Int64 msg_winch_target;
extern hero_msgs::hero_agent_cont_xy cont_xy_msg;

// Parameters loaded from YAML
extern double param_log_period;

// Winch model parameters
extern double winch_coeff_a, winch_coeff_b;
extern double winch_ticks_per_rev, winch_gear_num, winch_gear_den;

// Teleop
extern double teleop_xy_step, teleop_z_step;
extern int teleop_winch_step;
extern std::queue<int> key_input_queue;

// ==============================
// Inline utility functions
// ==============================

inline void updateWinchTarget(float cable_length) {
    if (std::abs(winch.pre_target_meter - cable_length) > 0.01) {
        winch.target_rotation = (winch_coeff_b - std::sqrt(winch_coeff_b * winch_coeff_b
            - 4 * cable_length * winch_coeff_a))
            / (2 * winch_coeff_a) / winch_gear_den * winch_gear_num;
        winch.target_position = winch.calib_position - winch.target_rotation * winch_ticks_per_rev;
        msg_winch_target.data = winch.target_position;
        pub_winch_target.publish(msg_winch_target);
        winch.pre_target_meter = cable_length;
    }
}

// ==============================
// Module function declarations
// ==============================

// darknet_control.cpp
void msgCallback_darknet(const darknet_ros_msgs::ObjectCount::ConstPtr &msg);
void msgCallback_darknet_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
void executeDarknetControl(int count);

// mosaic_survey.cpp
void executeMosaicSurvey(int count);

// teleop.cpp
void init_keyboard();
void close_keyboard();
void handleKeyboardInput(ros::Rate& loop_rate);

// Callbacks
void msgCallback_result(const hero_msgs::hero_agent_position_result::ConstPtr &msg);
void msgCallback_winch_pos(const std_msgs::Int64::ConstPtr &msg);

#endif // HERO_AGENT_TYPES_H
