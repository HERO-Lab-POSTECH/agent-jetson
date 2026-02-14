#include "hero_agent/hero_agent_types.h"

// ==============================
// Global variable definitions (struct-based)
// ==============================

NavigationState navi;
TargetState target;
WinchState winch;
QRState qr;
QRGains qr_gains;
DarknetState darknet;
MosaicState mosaic;
ControlFlags ctrl;
AutoRecoveryState auto_recovery;
ThrustOutput thrust;

// ROS publishers
ros::Publisher pub_command;
ros::Publisher pub_target;
ros::Publisher pub_winch_target;
ros::Publisher pub_cont_xy;
ros::Publisher pub_agent_qr_result;
ros::Publisher pub_key_input;

// ROS messages
std_msgs::Int8 command_msg;
hero_msgs::hero_agent_dvl msg_target;
std_msgs::Int64 msg_winch_target;
hero_msgs::hero_agent_cont_xy cont_xy_msg;
hero_msgs::hero_agent_position_result agent_qr_result_msg;

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
    navi.x = msg->X;
    navi.y = msg->Y;
    navi.z = msg->Z;
}

void key_input_callback(const std_msgs::Int8::ConstPtr &msg) {
    key_input_queue.push(msg->data);
}

void msgCallback_winch_pos(const std_msgs::Int64::ConstPtr &msg)
{
    winch.current_position = msg->data;
}

// ==============================
// Parameter loading from YAML
// ==============================

static void loadParameters(ros::NodeHandle& nh)
{
    // QR control gains
    nh.param<float>("qr_control/Kp", qr_gains.Kp, 0.005f);
    nh.param<float>("qr_control/Kd", qr_gains.Kd, 0.005f);
    nh.param<float>("qr_control/qr_Kp", qr_gains.qr_Kp, 65.0f);
    nh.param<float>("qr_control/qr_Kd", qr_gains.qr_Kd, 65.0f);
    nh.param<float>("qr_control/qr_Ki", qr_gains.qr_Ki, 0.05f);
    nh.param<float>("qr_control/qr_Mb", qr_gains.Mb, 0.16f);
    nh.param<float>("qr_control/qr_KKp", qr_gains.KKp, 5.0f);
    nh.param<float>("qr_control/qr_KKv", qr_gains.KKv, 26.0f);
    nh.param<double>("qr_control/saturation_default", param_saturation_default, 200.0);
    nh.param<double>("qr_control/saturation_recovery", param_saturation_recovery, 100.0);

    // DARKNET
    nh.param<float>("darknet/Kp", darknet.Kp, 0.1f);
    nh.param<float>("darknet/Kd", darknet.Kd, 0.1f);
    nh.param<float>("darknet/x_target", darknet.x_target, 320.0f);
    nh.param<float>("darknet/y_target", darknet.y_target, 300.0f);
    nh.param<double>("darknet/saturation", darknet.saturation, 100.0);

    // Mosaic
    nh.param<int>("mosaic/sway_num", mosaic.sway_num, 300);
    nh.param<int>("mosaic/surge_num", mosaic.surge_num, 50);
    nh.param<float>("mosaic/move_dis", mosaic.move_dis, 0.01f);

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

    fout << time_count << '\t' << qr.x_target << '\t' << qr.y_target << '\t'
         << qr.z_target << '\t' << qr.x << '\t' << qr.y << '\t' << qr.z << '\t'
         << thrust.Tx << '\t' << thrust.Ty << std::endl;

    if (time_count == 4000) {
        resetQrErrors();
        ctrl.qr_tdc = 1;
    } else if (time_count > 6000) {
        start_record = 0;
        time_count = 0;
        ctrl.recovery = -2;
        qr.flag_count = 0;
        ctrl.qr_tdc = 0;
        resetQrErrors();
    }
}

// ==============================
// Automated recovery sequence FSM
// ==============================

static void handleAutoRecovery(ros::Rate& loop_rate)
{
    if (auto_recovery.active != 1) return;

    auto_recovery.count++;

    // [BUG FIX H2] Reset count on all state transitions (done in switch below)

    if (auto_recovery.step == 0) {
        mosaic.start_position = winch.current_position;
        if (auto_recovery.count > 300) auto_recovery.step = 1;
    }
    else if (auto_recovery.step == 1 && auto_recovery.count > 300) {
        auto_recovery.step = 2;
    }
    else if (auto_recovery.step == 2 && auto_recovery.count > 300 && ctrl.recovery == 0) {
        auto_recovery.step = 3;
    }
    else if (auto_recovery.step == 3) {
        if (target.z < -0.55) auto_recovery.step = 4;
        else if (std::abs(navi.z - target.z) < 0.02 || auto_recovery.count > 100) {
            auto_recovery.step_pre = 2; auto_recovery.step = 3;
        }
    }
    else if (auto_recovery.step == 4) {
        if (std::abs(winch.target_position - winch.current_position) < 50 || std::abs(navi.x - target.x) < 0.02) {
            if (darknet.found >= 1) auto_recovery.step = 5;
            else { auto_recovery.step_pre = 3; auto_recovery.step = 4; }
        }
    }
    else if (auto_recovery.step == 5 && auto_recovery.count > 300) {
        winch.target_meter = std::sqrt(navi.x*navi.x + navi.y*navi.y + navi.z*navi.z);
        updateWinchTarget(winch.target_meter);
        auto_recovery.step = 6;
    }
    else if (auto_recovery.step == 6) {
        if (auto_recovery.count > 800) { ctrl.darknet = 0; auto_recovery.step = 7; }
        else if (auto_recovery.count >= 100 && std::abs(navi.z - target.z) < 0.03) {
            winch.target_meter = std::sqrt(navi.x*navi.x + navi.y*navi.y + navi.z*navi.z);
            updateWinchTarget(winch.target_meter);
            auto_recovery.step_pre = 5; auto_recovery.step = 6;
        }
    }
    else if (auto_recovery.step == 7 && auto_recovery.count > 300) {
        auto_recovery.step = 8;
    }
    else if (auto_recovery.step == 8) {
        if (target.z < -0.55) auto_recovery.step = 9;
        else if (std::abs(navi.z - target.z) < 0.02 || auto_recovery.count > 100) {
            auto_recovery.step_pre = 7; auto_recovery.step = 8;
        }
    }
    else if (auto_recovery.step == 9) {
        if (auto_recovery.count >= 100 &&
            (std::abs(winch.target_position - winch.current_position) < 50 || std::abs(navi.x - target.x) < 0.02)) {
            if ((qr.valid == 1 && qr.z < 1) || (target.x >= 0 && target.z > 0))
                auto_recovery.step = 10;
            else { auto_recovery.step_pre = 8; auto_recovery.step = 9; }
        }
    }
    else if (auto_recovery.step == 10 && auto_recovery.count > 300 && ctrl.recovery == 0) {
        auto_recovery.step = 11;
    }
    else if (auto_recovery.step == 11 && auto_recovery.count > 300 && ctrl.recovery == 0) {
        auto_recovery.active = 0; auto_recovery.count = 0;
        auto_recovery.step = 0; auto_recovery.step_pre = 0;
    }

    // State transition actions
    if (auto_recovery.step != auto_recovery.step_pre) {
        auto sendCmd = [&](char cmd) {
            command_msg.data = cmd;
            pub_command.publish(command_msg);
            loop_rate.sleep();
        };

        // [BUG FIX H2] Always reset count on state transition
        auto_recovery.count = 0;

        switch (auto_recovery.step) {
        case 1:
            target.z -= 0.05;
            sendCmd('n'); sendCmd('s');
            break;
        case 2:
            sendCmd('1');
            msg_target.command = 1;
            msg_target.TARGET_X = target.x; msg_target.TARGET_Y = target.y; msg_target.TARGET_Z = target.z;
            pub_target.publish(msg_target); loop_rate.sleep();
            msg_target.command = 0;
            target.x = 0; target.y = 0; target.z = 0; target.yaw = 0;
            winch.qr_based_calibration = 0;
            ctrl.recovery = 1; qr.flag_count = 0;
            break;
        case 3:
            sendCmd('4');
            msg_target.command = 0;
            target.z -= 0.02;
            break;
        case 4:
            target.x -= 0.03;
            break;
        case 5:
            sendCmd('1'); sendCmd('1'); sendCmd('1');
            ctrl.darknet = 1; mosaic.deep_count = 0;
            break;
        case 6:
            mosaic.deep_count++;
            target.z += 0.01;
            qr.flag_count = 0;
            break;
        case 7:
            sendCmd('b'); sendCmd('b'); sendCmd('b');
            break;
        case 8:
            target.z -= 0.02;
            break;
        case 9:
            sendCmd('4');
            if (target.x < 0) target.x += 0.03;
            if (target.z < 0) target.z += 0.02;
            break;
        case 10:
            sendCmd('1');
            winch.target_position = mosaic.start_position;
            msg_winch_target.data = winch.target_position;
            pub_winch_target.publish(msg_winch_target); loop_rate.sleep();
            msg_target.command = 0;
            ctrl.recovery = 1; qr.flag_count = 0;
            break;
        case 11:
            msg_target.command = 0;
            ctrl.recovery = 2; qr.flag_count = 0;
            break;
        }
        auto_recovery.step_pre = auto_recovery.step;
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
    pub_key_input = nh.advertise<std_msgs::Int8>("/hero_agent/key_input", 10);

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

    // Subscribe to key_input topic
    ros::Subscriber sub_key_input = nh.subscribe("/hero_agent/key_input", 10, key_input_callback);

    // Initialize stdin keyboard (raw mode, no echo)
    init_keyboard();

    // Startup banner
    ROS_INFO("===================================");
    ROS_INFO("  Agent Command Node Initialized");
    ROS_INFO("  QR Kp=%.3f Kd=%.3f Sat=%.0f/%.0f",
             qr_gains.Kp, qr_gains.Kd, param_saturation_default, param_saturation_recovery);
    ROS_INFO("  qr_Kp=%.1f qr_Kd=%.1f qr_Ki=%.2f", qr_gains.qr_Kp, qr_gains.qr_Kd, qr_gains.qr_Ki);
    ROS_INFO("  Controller: %s", ctrl.qr_tdc == 1 ? "TDC" : "PID");
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
        if (winch.qr_based_calibration == 1 && ctrl.recovery == 0 && ctrl.darknet == 0) {
            winch.target_meter = std::sqrt(target.x*target.x + target.y*target.y + target.z*target.z);
            updateWinchTarget(winch.target_meter);
        }

        // Publish target if changed (command=0 for incremental updates)
        if (pre_x != target.x || pre_y != target.y || pre_z != target.z || pre_yaw != target.yaw) {
            msg_target.command = 0;
            msg_target.TARGET_X = target.x;
            msg_target.TARGET_Y = target.y;
            msg_target.TARGET_Z = target.z;
            pub_target.publish(msg_target);
        }

        pre_x = target.x; pre_y = target.y; pre_z = target.z; pre_yaw = target.yaw;

        // Status logging (DEBUG level to avoid interfering with agent_main dashboard)
        ROS_DEBUG_THROTTLE(param_log_period,
            "rec=%d dk=%d mosaic=%d | Pos(%.2f,%.2f,%.2f) Tgt(%.2f,%.2f,%.2f) | Tx=%.1f Ty=%.1f | Winch=%ld",
            ctrl.recovery, ctrl.darknet, ctrl.mosaic,
            navi.x, navi.y, navi.z, target.x, target.y, target.z,
            thrust.Tx, thrust.Ty, (long)winch.current_position);

        loop_rate.sleep();
        ros::spinOnce();
    }

    close_keyboard();
    fout.close();
    return 0;
}
