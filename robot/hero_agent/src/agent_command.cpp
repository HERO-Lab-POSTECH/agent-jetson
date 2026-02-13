#include "hero_agent/agent_command_types.h"

// ==============================
// Global variable definitions
// ==============================

// Navigation state
float navi_x = 0, navi_y = 0, navi_z = 0, navi_yaw = 0;
float target_x = 0, target_y = 0, target_z = 0, target_yaw = 0;

// Winch state
int64_t current_position = 0, target_position = 0, calib_position = 0;
float target_meter = 0, pre_target_meter = 0, target_rotation = 0;
int qr_based_calibration = 0;

// ROS publishers
ros::Publisher pub_command;
ros::Publisher pub_target;
ros::Publisher pub_winch_target;
ros::Publisher pub_cont_xy;
ros::Publisher pub_agent_qr_result;

// ROS messages
std_msgs::Int8 command_msg;
hero_msgs::hero_agent_dvl msg_target;
std_msgs::Int64 msg_winch_target;
hero_msgs::hero_agent_cont_xy cont_xy_msg;
hero_msgs::hero_agent_position_result agent_qr_result_msg;

// DARKNET state
int found_darknet = 0;
int darknet_x = 0, darknet_y = 0, darknet_width = 0, darknet_height = 0, darknet_area = 0;

// Recovery state
int start_recovery = 0, start_count = 0, start_step = 0, start_step_pre = 0;
int cont_darknet = 0, cont_recovery = 0, cont_mosaic = 0;
int cont_qr_tdc = 0;

// QR state
float qr_x = 0, qr_y = 0, qr_z = 0, qr_yaw = 0;
int qr_valid = 0;
float pre_qr_x = 0, pre_qr_y = 0, pre_qr_z = 0;
float qr_x_target = 0, qr_y_target = 0, qr_z_target = 0, qr_yaw_target = 0;
float qr_x_error = 0, qr_y_error = 0, qr_z_error = 0, qr_yaw_error = 0;
float qr_x_error_pre = 0, qr_y_error_pre = 0, qr_z_error_pre = 0, qr_yaw_error_pre = 0;
float qr_x_error_d = 0, qr_y_error_d = 0, qr_z_error_d = 0, qr_yaw_error_d = 0;
float qr_x_error_d_pre = 0, qr_z_error_d_pre = 0;
float qr_x_error_sum = 0, qr_z_error_sum = 0;
float qr_x_ax = 0, qr_z_az = 0, qr_x_ax_pre = 0, qr_z_az_pre = 0;
float qr_Mb = 0.16f, qr_KKp = 5.0f, qr_KKv = 26.0f;
float Kp = 0.005f, Kd = 0.005f;
float qr_Kp = 65.0f, qr_Kd = 65.0f, qr_Ki = 0.05f;
double Tx = 0, Ty = 0;
int qr_count = 0, flag_count = 0;

// Mosaic state
int sway_count = 0, sway_num = 300, surge_count = 0, surge_num = 50;
float move_dis = 0.01f;
int deep_count = 0, start_position = 0;

// DARKNET control
float darknet_x_target = 320.0f, darknet_y_target = 300.0f;
float darknet_x_error = 0, darknet_y_error = 0;
float darknet_x_error_pre = 0, darknet_y_error_pre = 0;
float darknet_x_error_d = 0, darknet_y_error_d = 0;
float darknet_Kp = 0.1f, darknet_Kd = 0.1f;

// Recording
int time_count = 0, start_record = 0;
std::ofstream fout;

// Parameters (loaded from YAML, defaults match original hardcoded values)
double param_saturation_default = 200.0;
double param_saturation_recovery = 100.0;
double param_convergence_tolerance = 0.02;
int param_convergence_count = 30;
double param_qr_distance_threshold = 0.5;
int param_qr_warmup_count = 4;
double param_log_period = 0.5;

// Winch model parameters
double winch_coeff_a = 0.00171073;
double winch_coeff_b = 0.534071;
double winch_ticks_per_rev = 4096;
double winch_gear_num = 435.45;
double winch_gear_den = 73.45613;

// Teleop step sizes
double teleop_xy_step = 0.05;
double teleop_z_step = 0.01;
int teleop_winch_step = 10000;
std::queue<int> key_input_queue;

// ==============================
// Simple callbacks
// ==============================

void msgCallback_result(const hero_msgs::hero_agent_position_result::ConstPtr &msg)
{
    navi_x = msg->X;
    navi_y = msg->Y;
    navi_z = msg->Z;
}

void key_input_callback(const std_msgs::Int8::ConstPtr &msg) {
    key_input_queue.push(msg->data);
}

void msgCallback_winch_pos(const std_msgs::Int64::ConstPtr &msg)
{
    current_position = msg->data;
}

// Forward declaration
extern void initRecoveryConfigs();

// ==============================
// Parameter loading from YAML
// ==============================

static void loadParameters(ros::NodeHandle& nh)
{
    // QR control gains
    nh.param<float>("qr_control/Kp", Kp, 0.005f);
    nh.param<float>("qr_control/Kd", Kd, 0.005f);
    nh.param<float>("qr_control/qr_Kp", qr_Kp, 65.0f);
    nh.param<float>("qr_control/qr_Kd", qr_Kd, 65.0f);
    nh.param<float>("qr_control/qr_Ki", qr_Ki, 0.05f);
    nh.param<float>("qr_control/qr_Mb", qr_Mb, 0.16f);
    nh.param<float>("qr_control/qr_KKp", qr_KKp, 5.0f);
    nh.param<float>("qr_control/qr_KKv", qr_KKv, 26.0f);
    nh.param<double>("qr_control/saturation_default", param_saturation_default, 200.0);
    nh.param<double>("qr_control/saturation_recovery", param_saturation_recovery, 100.0);

    // DARKNET
    nh.param<float>("darknet/Kp", darknet_Kp, 0.1f);
    nh.param<float>("darknet/Kd", darknet_Kd, 0.1f);
    nh.param<float>("darknet/x_target", darknet_x_target, 320.0f);
    nh.param<float>("darknet/y_target", darknet_y_target, 300.0f);

    // Mosaic
    nh.param<int>("mosaic/sway_num", sway_num, 300);
    nh.param<int>("mosaic/surge_num", surge_num, 50);
    nh.param<float>("mosaic/move_dis", move_dis, 0.01f);

    // Winch model
    nh.param<double>("winch/coeff_a", winch_coeff_a, 0.00171073);
    nh.param<double>("winch/coeff_b", winch_coeff_b, 0.534071);
    nh.param<double>("winch/ticks_per_rev", winch_ticks_per_rev, 4096);
    nh.param<double>("winch/gear_ratio_num", winch_gear_num, 435.45);
    nh.param<double>("winch/gear_ratio_den", winch_gear_den, 73.45613);

    // Teleop
    nh.param<double>("teleop/xy_step", teleop_xy_step, 0.05);
    nh.param<double>("teleop/z_step", teleop_z_step, 0.01);
    nh.param<int>("teleop/winch_step", teleop_winch_step, 10000);

    // QR filter
    nh.param<double>("qr_filter/distance_threshold", param_qr_distance_threshold, 0.5);
    nh.param<int>("qr_filter/warmup_count", param_qr_warmup_count, 4);

    // Convergence
    nh.param<double>("convergence/position_tolerance", param_convergence_tolerance, 0.02);
    nh.param<int>("convergence/count_threshold", param_convergence_count, 30);

    // Logging
    nh.param<double>("log_period", param_log_period, 0.5);
}

// ==============================
// Recording logic
// ==============================

static void handleRecording()
{
    if (start_record != 1) return;

    time_count++;

    if (time_count >= 2000 && time_count < 6000) {
        // Disturbance window (placeholder)
    }

    fout << time_count << '\t' << qr_x_target << '\t' << qr_y_target << '\t'
         << qr_z_target << '\t' << qr_x << '\t' << qr_y << '\t' << qr_z << '\t'
         << Tx << '\t' << Ty << std::endl;

    if (time_count == 4000) {
        resetQrErrors();
        cont_qr_tdc = 1;
    } else if (time_count > 6000) {
        start_record = 0;
        time_count = 0;
        cont_recovery = -2;
        flag_count = 0;
        cont_qr_tdc = 0;
        resetQrErrors();
    }
}

// ==============================
// Automated recovery sequence FSM
// ==============================

static void handleAutoRecovery(ros::Rate& loop_rate)
{
    if (start_recovery != 1) return;

    start_count++;

    if (start_step == 0) {
        start_position = current_position;
        if (start_count > 300) start_step = 1;
    }
    else if (start_step == 1 && start_count > 300) { start_step = 2; }
    else if (start_step == 2 && start_count > 300 && cont_recovery == 0) { start_step = 3; }
    else if (start_step == 3) {
        if (target_z < -0.55) start_step = 4;
        else if (std::abs(navi_z - target_z) < 0.02 || start_count > 100) {
            start_step_pre = 2; start_step = 3;
        }
    }
    else if (start_step == 4) {
        if (std::abs(target_position - current_position) < 50 || std::abs(navi_x - target_x) < 0.02) {
            if (found_darknet >= 1) start_step = 5;
            else { start_step_pre = 3; start_step = 4; }
        }
    }
    else if (start_step == 5 && start_count > 300) {
        target_meter = std::sqrt(navi_x*navi_x + navi_y*navi_y + navi_z*navi_z);
        updateWinchTarget(target_meter);
        start_step = 6;
    }
    else if (start_step == 6) {
        if (start_count > 800) { cont_darknet = 0; start_step = 7; }
        else if (start_count >= 100 && std::abs(navi_z - target_z) < 0.03) {
            target_meter = std::sqrt(navi_x*navi_x + navi_y*navi_y + navi_z*navi_z);
            updateWinchTarget(target_meter);
            start_step_pre = 5; start_step = 6;
        }
    }
    else if (start_step == 7 && start_count > 300) { start_step = 8; }
    else if (start_step == 8) {
        if (target_z < -0.55) start_step = 9;
        else if (std::abs(navi_z - target_z) < 0.02 || start_count > 100) {
            start_step_pre = 7; start_step = 8;
        }
    }
    else if (start_step == 9) {
        if (start_count >= 100 &&
            (std::abs(target_position - current_position) < 50 || std::abs(navi_x - target_x) < 0.02)) {
            if ((qr_valid == 1 && qr_z < 1) || (target_x >= 0 && target_z > 0))
                start_step = 10;
            else { start_step_pre = 8; start_step = 9; }
        }
    }
    else if (start_step == 10 && start_count > 300 && cont_recovery == 0) { start_step = 11; }
    else if (start_step == 11 && start_count > 300 && cont_recovery == 0) {
        start_recovery = 0; start_count = 0; start_step = 0; start_step_pre = 0;
    }

    // State transition actions
    if (start_step != start_step_pre) {
        auto sendCmd = [&](char cmd) {
            command_msg.data = cmd;
            pub_command.publish(command_msg);
            loop_rate.sleep();
            ros::spinOnce();
        };

        switch (start_step) {
        case 1:
            target_z -= 0.05;
            sendCmd('n'); sendCmd('s');
            start_count = 0;
            break;
        case 2:
            sendCmd('1');
            msg_target.command = 1;
            msg_target.TARGET_X = target_x; msg_target.TARGET_Y = target_y; msg_target.TARGET_Z = target_z;
            pub_target.publish(msg_target); loop_rate.sleep(); ros::spinOnce();
            msg_target.command = 0;
            start_count = 0;
            target_x = 0; target_y = 0; target_z = 0; target_yaw = 0;
            qr_based_calibration = 0;
            cont_recovery = 1; flag_count = 0;
            break;
        case 3:
            sendCmd('4');
            start_count = 0; msg_target.command = 0;
            target_z -= 0.02;
            break;
        case 4:
            target_x -= 0.03;
            start_count = 0;
            break;
        case 5:
            sendCmd('1'); sendCmd('1'); sendCmd('1');
            start_count = 0; cont_darknet = 1; deep_count = 0;
            break;
        case 6:
            deep_count++;
            target_z += 0.01;
            start_count = 0; flag_count = 0;
            break;
        case 7:
            sendCmd('b'); sendCmd('b'); sendCmd('b');
            start_count = 0;
            break;
        case 8:
            target_z -= 0.02;
            start_count = 0;
            break;
        case 9:
            sendCmd('4');
            if (target_x < 0) target_x += 0.03;
            if (target_z < 0) target_z += 0.02;
            start_count = 0;
            break;
        case 10:
            sendCmd('1');
            target_position = start_position;
            msg_winch_target.data = target_position;
            pub_winch_target.publish(msg_winch_target); loop_rate.sleep(); ros::spinOnce();
            msg_target.command = 0;
            cont_recovery = 1; flag_count = 0; start_count = 0;
            break;
        case 11:
            msg_target.command = 0;
            cont_recovery = 2; flag_count = 0; start_count = 0;
            break;
        }
        start_step_pre = start_step;
    }
}

// ==============================
// Main
// ==============================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent_command");
    ros::NodeHandle nh("~");

    // Load parameters from YAML (or use defaults)
    int loop_rate_hz;
    nh.param<int>("loop_rate_hz", loop_rate_hz, 100);
    loadParameters(nh);

    ros::Rate loop_rate(loop_rate_hz);

    // Initialize recovery mode config table
    initRecoveryConfigs();

    // Publishers
    pub_command = nh.advertise<std_msgs::Int8>("/hero_agent/command", 100);
    pub_target = nh.advertise<hero_msgs::hero_agent_dvl>("/hero_agent/dvl", 100);
    pub_winch_target = nh.advertise<std_msgs::Int64>("/winch/target", 100);
    pub_cont_xy = nh.advertise<hero_msgs::hero_agent_cont_xy>("/hero_agent/cont_xy_darknet", 100);
    pub_agent_qr_result = nh.advertise<hero_msgs::hero_agent_position_result>("/hero_agent/qr_result", 100);

    // Subscribers
    ros::Subscriber sub_winch_pos = nh.subscribe("/winch/pos", 100, msgCallback_winch_pos);
    ros::Subscriber sub_result = nh.subscribe("/hero_agent/result", 100, msgCallback_result);
    ros::Subscriber sub_ip_qr = nh.subscribe("/hero_ipcam_qr_msg", 100, msgCallback_ip_qr);
    ros::Subscriber sub_darknet_box = nh.subscribe("/darknet_ros/bounding_boxes_object", 100, msgCallback_darknet_box);
    ros::Subscriber sub_darknet = nh.subscribe("/darknet_ros/found_object_object", 100, msgCallback_darknet);
    ros::Subscriber sub_cont_para = nh.subscribe("/hero_agent/qr_cont_para", 100, msgCallback_cont_para);

    // Open log file (parameterized path)
    std::string log_file_path;
    nh.param<std::string>("log_file_path", log_file_path, "/home/nvidia/catkin_ws/agent_results/tdc_out.txt");
    fout.open(log_file_path);

    init_keyboard();
    ros::Subscriber sub_key_input = nh.subscribe("/hero_agent/key_input", 10, key_input_callback);

    // Startup banner
    ROS_INFO("===================================");
    ROS_INFO("  Agent Command Node Initialized");
    ROS_INFO("  QR Kp=%.3f Kd=%.3f Sat=%.0f/%.0f",
             Kp, Kd, param_saturation_default, param_saturation_recovery);
    ROS_INFO("  qr_Kp=%.1f qr_Kd=%.1f qr_Ki=%.2f", qr_Kp, qr_Kd, qr_Ki);
    ROS_INFO("  Controller: %s", cont_qr_tdc == 1 ? "TDC" : "PID");
    ROS_INFO("  Loop rate: %d Hz", loop_rate_hz);
    ROS_INFO("===================================");

    float pre_x = 0, pre_y = 0, pre_z = 0, pre_yaw = 0;
    int count = 0;

    while (ros::ok())
    {
        count = (count + 1) % 10;

        // Control modules
        executeDarknetControl(count);
        executeMosaicSurvey(count);
        handleRecording();
        handleAutoRecovery(loop_rate);
        handleKeyboardInput(loop_rate);

        // Winch auto-update from QR calibration
        if (qr_based_calibration == 1 && cont_recovery == 0 && cont_darknet == 0) {
            target_meter = std::sqrt(target_x*target_x + target_y*target_y + target_z*target_z);
            updateWinchTarget(target_meter);
        }

        // Publish target if changed
        if (pre_x != target_x || pre_y != target_y || pre_z != target_z || pre_yaw != target_yaw) {
            msg_target.TARGET_X = target_x;
            msg_target.TARGET_Y = target_y;
            msg_target.TARGET_Z = target_z;
            pub_target.publish(msg_target);
            loop_rate.sleep();
            ros::spinOnce();
        }

        pre_x = target_x; pre_y = target_y; pre_z = target_z; pre_yaw = target_yaw;

        // Status logging (DEBUG level to avoid polluting agent_main dashboard)
        ROS_DEBUG_THROTTLE(param_log_period,
            "cont_recovery=%d darknet=%d mosaic=%d | Pos(%.2f,%.2f,%.2f) Tgt(%.2f,%.2f,%.2f) | Tx=%.1f Ty=%.1f | Winch=%ld",
            cont_recovery, cont_darknet, cont_mosaic,
            navi_x, navi_y, navi_z, target_x, target_y, target_z,
            Tx, Ty, (long)current_position);

        loop_rate.sleep();
        ros::spinOnce();
    }

    fout.close();
    close_keyboard();
    return 0;
}
