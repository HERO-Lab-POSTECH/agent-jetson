#include "hero_agent/agent_command_types.h"

// ==============================
// DARKNET visual servoing control
// ==============================

void msgCallback_darknet(const darknet_ros_msgs::ObjectCount::ConstPtr &msg)
{
    found_darknet = msg->count;
}

void msgCallback_darknet_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    if (msg->bounding_boxes.empty()) return;
    darknet_ros_msgs::BoundingBox input = msg->bounding_boxes[0];
    darknet_x = (input.xmin + input.xmax) / 2;
    darknet_y = (input.ymin + input.ymax) / 2;
    darknet_width = input.xmax - input.xmin;
    darknet_height = input.ymax - input.ymin;
    darknet_area = darknet_width * darknet_height;
}

void executeDarknetControl(int count)
{
    if (cont_darknet != 1) return;

    darknet_x_error = darknet_x_target - static_cast<float>(darknet_x);
    darknet_y_error = darknet_y_target - static_cast<float>(darknet_y);

    darknet_x_error_d = darknet_x_error - darknet_x_error_pre;
    darknet_y_error_d = darknet_y_error - darknet_y_error_pre;

    darknet_x_error_pre = darknet_x_error;
    darknet_y_error_pre = darknet_y_error;

    Ty = -darknet_Kp * darknet_x_error - darknet_Kd * darknet_x_error_d;
    Tx =  darknet_Kp * darknet_y_error + darknet_Kd * darknet_y_error_d;

    const double sat = 100.0;
    Tx = clamp(Tx, -sat, sat);
    Ty = clamp(Ty, -sat, sat);

    if (found_darknet >= 1) {
        cont_xy_msg.T0 = -Tx + Ty;
        cont_xy_msg.T1 = -Tx - Ty;
        cont_xy_msg.T2 =  Tx - Ty;
        cont_xy_msg.T3 =  Tx + Ty;
    } else {
        cont_xy_msg.T0 = 0;
        cont_xy_msg.T1 = 0;
        cont_xy_msg.T2 = 0;
        cont_xy_msg.T3 = 0;
    }
    cont_xy_msg.TARGET_DEPTH = 0;

    pub_cont_xy.publish(cont_xy_msg);
    ros::spinOnce();
}
