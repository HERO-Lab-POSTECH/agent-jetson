#include "hero_agent/hero_agent_types.h"

// ==============================
// DARKNET visual servoing control
// ==============================

void msgCallback_darknet(const darknet_ros_msgs::ObjectCount::ConstPtr &msg)
{
    darknet.found = msg->count;
}

void msgCallback_darknet_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    if (msg->bounding_boxes.empty()) return;
    darknet_ros_msgs::BoundingBox input = msg->bounding_boxes[0];
    darknet.x = (input.xmin + input.xmax) / 2;
    darknet.y = (input.ymin + input.ymax) / 2;
    darknet.width = input.xmax - input.xmin;
    darknet.height = input.ymax - input.ymin;
    darknet.area = darknet.width * darknet.height;
}

void executeDarknetControl(int count)
{
    if (ctrl.darknet != 1) return;

    darknet.x_error = darknet.x_target - static_cast<float>(darknet.x);
    darknet.y_error = darknet.y_target - static_cast<float>(darknet.y);

    darknet.x_error_d = darknet.x_error - darknet.x_error_pre;
    darknet.y_error_d = darknet.y_error - darknet.y_error_pre;

    darknet.x_error_pre = darknet.x_error;
    darknet.y_error_pre = darknet.y_error;

    thrust.Ty = -darknet.Kp * darknet.x_error - darknet.Kd * darknet.x_error_d;
    thrust.Tx =  darknet.Kp * darknet.y_error + darknet.Kd * darknet.y_error_d;

    // [BUG FIX H3] Use YAML parameter instead of hardcoded value
    const double sat = darknet.saturation;
    thrust.Tx = clamp(thrust.Tx, -sat, sat);
    thrust.Ty = clamp(thrust.Ty, -sat, sat);

    if (darknet.found >= 1) {
        cont_xy_msg.T0 = -thrust.Tx + thrust.Ty;
        cont_xy_msg.T1 = -thrust.Tx - thrust.Ty;
        cont_xy_msg.T2 =  thrust.Tx - thrust.Ty;
        cont_xy_msg.T3 =  thrust.Tx + thrust.Ty;
    } else {
        cont_xy_msg.T0 = 0;
        cont_xy_msg.T1 = 0;
        cont_xy_msg.T2 = 0;
        cont_xy_msg.T3 = 0;
    }
    cont_xy_msg.TARGET_DEPTH = 0;

    pub_cont_xy.publish(cont_xy_msg);
    // [BUG FIX C3-related] Removed ros::spinOnce() - not needed here
}
