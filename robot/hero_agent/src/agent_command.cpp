#include "hero_agent/hero_agent_types.h"

// ==============================
// Global variable definitions (struct-based)
// ==============================

NavigationState navi;
TargetState target;
WinchState winch;
DarknetState darknet;
MosaicState mosaic;
ControlFlags ctrl;
ThrustOutput thrust;

// ROS publishers
ros::Publisher pub_command;
ros::Publisher pub_target;
ros::Publisher pub_winch_target;
ros::Publisher pub_cont_xy;
ros::Publisher pub_key_input;

// ROS messages
std_msgs::Int8 command_msg;
hero_msgs::hero_agent_dvl msg_target;
std_msgs::Int64 msg_winch_target;
hero_msgs::hero_agent_cont_xy cont_xy_msg;

// Parameters (loaded from YAML, defaults match original hardcoded values)
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

void key_translated_callback(const std_msgs::Int8::ConstPtr &msg) {
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

    // Logging
    nh.param<double>("log_period", param_log_period, 0.5);
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

    // Publishers
    pub_command = nh.advertise<std_msgs::Int8>("/hero_agent/command", 100);
    pub_target = nh.advertise<hero_msgs::hero_agent_dvl>("/hero_agent/dvl", 100);
    pub_winch_target = nh.advertise<std_msgs::Int64>("/winch/target", 100);
    pub_cont_xy = nh.advertise<hero_msgs::hero_agent_cont_xy>("/hero_agent/cont_xy_darknet", 100);
    pub_key_input = nh.advertise<std_msgs::Int8>("/hero_agent/key_input", 10);

    // Subscribers
    ros::Subscriber sub_winch_pos = nh.subscribe("/winch/pos", 100, msgCallback_winch_pos);
    ros::Subscriber sub_result = nh.subscribe("/hero_agent/result", 100, msgCallback_result);
    ros::Subscriber sub_darknet_box = nh.subscribe("/darknet_ros/bounding_boxes_object", 100, msgCallback_darknet_box);
    ros::Subscriber sub_darknet = nh.subscribe("/darknet_ros/found_object_object", 100, msgCallback_darknet);

    // Subscribe to translated key topic (from agent_main V3 translation layer)
    ros::Subscriber sub_key_translated = nh.subscribe("/hero_agent/key_translated", 10, key_translated_callback);

    // Initialize stdin keyboard (raw mode, no echo)
    init_keyboard();

    // Startup banner
    ROS_INFO("===================================");
    ROS_INFO("  Agent Command Node Initialized");
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
        handleKeyboardInput(loop_rate);

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
            "dk=%d mosaic=%d | Pos(%.2f,%.2f,%.2f) Tgt(%.2f,%.2f,%.2f) | Winch=%ld",
            ctrl.darknet, ctrl.mosaic,
            navi.x, navi.y, navi.z, target.x, target.y, target.z,
            (long)winch.current_position);

        loop_rate.sleep();
        ros::spinOnce();
    }

    close_keyboard();
    return 0;
}
