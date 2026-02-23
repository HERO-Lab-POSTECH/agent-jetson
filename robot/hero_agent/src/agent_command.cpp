#include "hero_agent/hero_agent_types.h"

// ==============================
// Global variable definitions (struct-based)
// ==============================

TargetState target;
DarknetState darknet;
LawnmowerState lawnmower;
ControlFlags ctrl;
ThrustOutput thrust;

// ROS publishers
ros::Publisher pub_command;
ros::Publisher pub_target;
ros::Publisher pub_cont_xy;
ros::Publisher pub_key_input;

// ROS messages
std_msgs::Int8 command_msg;
hero_msgs::hero_agent_dvl msg_target;
hero_msgs::hero_agent_cont_xy cont_xy_msg;

// Parameters (loaded from YAML, defaults match original hardcoded values)
double param_log_period = 0.5;

// Teleop step sizes
double teleop_xy_step = 0.05;
double teleop_z_step = 0.01;
std::queue<int> key_input_queue;

// ==============================
// Simple callbacks
// ==============================

void key_translated_callback(const std_msgs::Int8::ConstPtr &msg) {
    key_input_queue.push(msg->data);
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

    // Lawnmower
    nh.param<int>("lawnmower/sway_num", lawnmower.sway_num, 300);
    nh.param<int>("lawnmower/surge_num", lawnmower.surge_num, 50);
    nh.param<float>("lawnmower/move_dis", lawnmower.move_dis, 0.01f);

    // Teleop
    nh.param<double>("teleop/xy_step", teleop_xy_step, 0.05);
    nh.param<double>("teleop/z_step", teleop_z_step, 0.01);

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
    pub_cont_xy = nh.advertise<hero_msgs::hero_agent_cont_xy>("/hero_agent/cont_xy_darknet", 100);
    pub_key_input = nh.advertise<std_msgs::Int8>("/hero_agent/key_input", 10);

    // Subscribers
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
        executeLawnmowerSurvey(count);
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
            "dk=%d lm=%d | Tgt(%.2f,%.2f,%.2f)",
            ctrl.darknet, ctrl.lawnmower,
            target.x, target.y, target.z);

        loop_rate.sleep();
        ros::spinOnce();
    }

    close_keyboard();
    return 0;
}
