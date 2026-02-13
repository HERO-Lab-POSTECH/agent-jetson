#ifndef AGENT_COMMAND_TYPES_H
#define AGENT_COMMAND_TYPES_H

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

/**
 * @brief Clamp a value to [min_val, max_val].
 */
template <typename T>
inline T clamp(T value, T min_val, T max_val) {
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

// ==============================
// QR Recovery Mode Configuration
// ==============================

struct RecoveryModeConfig {
    float x_target;
    float y_target;
    float z_target;
    double saturation;          // Thrust saturation limit
    bool yaw_correction;        // Enable yaw correction via commands
    bool accumulate_target;     // Use += for target_x/y/z (modes 2,3)
    int next_mode;              // Mode to transition to on convergence (0 = done)
    bool reset_on_converge;     // Reset targets and calibrate on convergence
};

// ==============================
// Shared state (extern declarations)
// ==============================

// Navigation state
extern float navi_x, navi_y, navi_z, navi_yaw;
extern float target_x, target_y, target_z, target_yaw;

// Winch state
extern int64_t current_position, target_position, calib_position;
extern float target_meter, pre_target_meter, target_rotation;
extern int qr_based_calibration;

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

// DARKNET state
extern int found_darknet;
extern int darknet_x, darknet_y, darknet_width, darknet_height, darknet_area;

// Recovery / control mode flags
extern int start_recovery, start_count, start_step, start_step_pre;
extern int cont_darknet, cont_recovery, cont_mosaic;
extern int cont_qr_tdc;

// QR state
extern float qr_x, qr_y, qr_z, qr_yaw;
extern int qr_valid;
extern float pre_qr_x, pre_qr_y, pre_qr_z;
extern float qr_x_target, qr_y_target, qr_z_target, qr_yaw_target;
extern float qr_x_error, qr_y_error, qr_z_error, qr_yaw_error;
extern float qr_x_error_pre, qr_y_error_pre, qr_z_error_pre, qr_yaw_error_pre;
extern float qr_x_error_d, qr_y_error_d, qr_z_error_d, qr_yaw_error_d;
extern float qr_x_error_d_pre, qr_z_error_d_pre;
extern float qr_x_error_sum, qr_z_error_sum;
extern float qr_x_ax, qr_z_az, qr_x_ax_pre, qr_z_az_pre;
extern float qr_Mb, qr_KKp, qr_KKv;
extern float Kp, Kd;
extern float qr_Kp, qr_Kd, qr_Ki;
extern double Tx, Ty;
extern int qr_count, flag_count;

// Mosaic state
extern int sway_count, sway_num, surge_count, surge_num;
extern float move_dis;
extern int deep_count, start_position;

// DARKNET control
extern float darknet_x_target, darknet_y_target;
extern float darknet_x_error, darknet_y_error;
extern float darknet_x_error_pre, darknet_y_error_pre;
extern float darknet_x_error_d, darknet_y_error_d;
extern float darknet_Kp, darknet_Kd;

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

// Teleop steps
extern double teleop_xy_step, teleop_z_step;
extern int teleop_winch_step;
extern std::queue<int> key_input_queue;

/**
 * @brief Reset all QR error tracking variables to zero.
 */
inline void resetQrErrors() {
    qr_x_error = 0; qr_y_error = 0; qr_z_error = 0;
    qr_x_error_d = 0; qr_y_error_d = 0; qr_z_error_d = 0;
    qr_x_error_d_pre = 0; qr_z_error_d_pre = 0;
    qr_x_ax = 0; qr_z_az = 0;
    Tx = 0; Ty = 0;
    qr_x_ax_pre = 0; qr_z_az_pre = 0;
    qr_x_error_sum = 0; qr_z_error_sum = 0;
    qr_x_error_pre = 0; qr_y_error_pre = 0;
    qr_z_error_pre = 0; qr_yaw_error_pre = 0;
    qr_count = 0;
}

/**
 * @brief Compute winch target from cable length using quadratic rope model.
 */
inline void updateWinchTarget(float cable_length) {
    if (std::abs(pre_target_meter - cable_length) > 0.01) {
        target_rotation = (winch_coeff_b - std::sqrt(winch_coeff_b * winch_coeff_b
            - 4 * cable_length * winch_coeff_a))
            / (2 * winch_coeff_a) / winch_gear_den * winch_gear_num;
        target_position = calib_position - target_rotation * winch_ticks_per_rev;
        msg_winch_target.data = target_position;
        pub_winch_target.publish(msg_winch_target);
        pre_target_meter = cable_length;
    }
}

// ==============================
// Module function declarations
// ==============================

// qr_recovery_control.cpp
void msgCallback_ip_qr(const ros_opencv_ipcam_qr::hero_ipcam_qr_msg::ConstPtr &msg);
void msgCallback_cont_para(const hero_msgs::hero_agent_cont_para::ConstPtr &msg);

// darknet_control.cpp
void msgCallback_darknet(const darknet_ros_msgs::ObjectCount::ConstPtr &msg);
void msgCallback_darknet_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
void executeDarknetControl(int count);

// mosaic_survey.cpp
void executeMosaicSurvey(int count);

// keyboard_handler.cpp
void handleKeyboardInput(ros::Rate& loop_rate);

// keyboard_utils.cpp
void init_keyboard();
void close_keyboard();
int _kbhit();
int _getch();
int _putch(int c);

// Callbacks
void msgCallback_result(const hero_msgs::hero_agent_position_result::ConstPtr &msg);
void msgCallback_winch_pos(const std_msgs::Int64::ConstPtr &msg);

#endif // AGENT_COMMAND_TYPES_H
