#ifndef HERO_AGENT_TYPES_H
#define HERO_AGENT_TYPES_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int64.h>
#include "hero_msgs/hero_agent_dvl.h"
#include "hero_msgs/hero_agent_position_result.h"
#include "hero_msgs/hero_agent_cont_xy.h"
#include "hero_msgs/hero_agent_cont_para.h"
#include "ros_opencv_ipcam_qr/hero_ipcam_qr_msg.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/ObjectCount.h"
#include "hero_msgs/hero_agent_sensor.h"

#include <cmath>
#include <fstream>
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
// FSM and mode enumerations
// ==============================

enum class RecoveryMode : int {
    Off       =  0,
    Approach  =  1,
    Close     =  2,
    Final     =  3,
    Deploy    =  4,
    ExpHold   = -1,
    ExpClose  = -2
};

// ==============================
// QR Recovery Mode Configuration
// ==============================

struct RecoveryModeConfig {
    float x_target;
    float y_target;
    float z_target;
    double saturation;
    bool yaw_correction;
    bool accumulate_target;
    int next_mode;
    bool reset_on_converge;
};

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
    int qr_based_calibration = 0;
};

struct QRState {
    float x = 0, y = 0, z = 0, yaw = 0;
    int valid = 0;
    float pre_x = 0, pre_y = 0, pre_z = 0;
    // Targets
    float x_target = 0, y_target = 0, z_target = 0, yaw_target = 0;
    // Errors
    float x_error = 0, y_error = 0, z_error = 0, yaw_error = 0;
    float x_error_pre = 0, y_error_pre = 0, z_error_pre = 0, yaw_error_pre = 0;
    float x_error_d = 0, y_error_d = 0, z_error_d = 0, yaw_error_d = 0;
    float x_error_d_pre = 0, z_error_d_pre = 0;
    float x_error_sum = 0, z_error_sum = 0;
    // Acceleration
    float x_ax = 0, z_az = 0, x_ax_pre = 0, z_az_pre = 0;
    // Count
    int count = 0;
    int flag_count = 0;
};

struct QRGains {
    float Mb = 0.16f, KKp = 5.0f, KKv = 26.0f;
    float Kp = 0.005f, Kd = 0.005f;
    float qr_Kp = 65.0f, qr_Kd = 65.0f, qr_Ki = 0.05f;
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
    int recovery = 0;
    int mosaic = 0;
    int qr_tdc = 0;
};

struct AutoRecoveryState {
    int active = 0;
    int count = 0;
    int step = 0;
    int step_pre = 0;
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
extern QRState qr;
extern QRGains qr_gains;
extern DarknetState darknet;
extern MosaicState mosaic;
extern ControlFlags ctrl;
extern AutoRecoveryState auto_recovery;
extern ThrustOutput thrust;

// ROS publishers
extern ros::Publisher pub_command;
extern ros::Publisher pub_target;
extern ros::Publisher pub_winch_target;
extern ros::Publisher pub_cont_xy;
extern ros::Publisher pub_agent_qr_result;

// ROS messages
extern std_msgs::Int8 command_msg;
extern hero_msgs::hero_agent_dvl msg_target;
extern std_msgs::Int64 msg_winch_target;
extern hero_msgs::hero_agent_cont_xy cont_xy_msg;
extern hero_msgs::hero_agent_position_result agent_qr_result_msg;

// Recording
extern int time_count, start_record;
extern std::ofstream fout;

// Parameters loaded from YAML
extern double param_saturation_default;
extern double param_saturation_recovery;
extern double param_convergence_tolerance;
extern int param_convergence_count;
extern double param_qr_distance_threshold;
extern int param_qr_warmup_count;
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

inline void resetQrErrors() {
    qr.x_error = 0; qr.y_error = 0; qr.z_error = 0;
    qr.x_error_d = 0; qr.y_error_d = 0; qr.z_error_d = 0;
    qr.x_error_d_pre = 0; qr.z_error_d_pre = 0;
    qr.x_ax = 0; qr.z_az = 0;
    thrust.Tx = 0; thrust.Ty = 0;
    qr.x_ax_pre = 0; qr.z_az_pre = 0;
    qr.x_error_sum = 0; qr.z_error_sum = 0;
    qr.x_error_pre = 0; qr.y_error_pre = 0;
    qr.z_error_pre = 0; qr.yaw_error_pre = 0;
    qr.count = 0;
}

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

// qr_recovery_control.cpp
void initRecoveryConfigs();
void msgCallback_ip_qr(const ros_opencv_ipcam_qr::hero_ipcam_qr_msg::ConstPtr &msg);
void msgCallback_cont_para(const hero_msgs::hero_agent_cont_para::ConstPtr &msg);

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
