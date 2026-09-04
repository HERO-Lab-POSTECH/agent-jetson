// Production header: StatusPublisher — owns the three output topics of
// albc_controller and encapsulates the joint-angle + /albc/status publish.
//
// Absorbs the former main()-local publishers (albc_controller.cpp):
//   - angle_pub_1 / angle_pub_2 : /albc/joint{1,2}_cmd
//   - status_pub                : /albc/status
// and the two publish blocks (joint angles + status) from the control loop.
//
// ⚠️ CONTRACT (must stay byte-identical):
//   - joint angle topics, type std_msgs/Float64, queue 1000, value mapTo2Pi(theta).
//   - /albc/status: std_msgs/Float64MultiArray, queue 10, 11 fields in the EXACT
//     order/units of the former status_msg.data:
//       0 RAD2DEG(target_roll)   1 RAD2DEG(current_roll)
//       2 RAD2DEG(target_pitch)  3 RAD2DEG(current_pitch)
//       4 target_x               5 target_y
//       6 current_x              7 current_y
//       8 angular_vel_roll       9 angular_vel_pitch   10 angular_vel_yaw
//     chain1's csv_logger reads these 11 fields in this order — NEVER reorder.
//
// ROS-dependent by design: owns ros::Publisher and builds std_msgs payloads, so
// it includes ros/ros.h + the message headers (unlike the pure oracle headers).

#ifndef ALBC_CONTROL_STATUS_PUBLISHER_H
#define ALBC_CONTROL_STATUS_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "hero_msgs/topics.h"
#include "albc_control/albc_kinematics.h"  // mapTo2Pi, RAD2DEG

namespace albc {

class StatusPublisher {
public:
    // Advertise the three output topics. Topic names, types, and queue sizes are
    // byte-identical to the former main() advertises (angle 1000, status 10).
    void advertise(ros::NodeHandle& nh) {
        angle_pub_1_ = nh.advertise<std_msgs::Float64>(
            hero_msgs::topics::JOINT1_CMD, 1000);
        angle_pub_2_ = nh.advertise<std_msgs::Float64>(
            hero_msgs::topics::JOINT2_CMD, 1000);
        status_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
            hero_msgs::topics::ALBC_STATUS, 10);
    }

    // Publish the two joint angles. Byte-identical to the former block:
    //   ros_theta1.data = mapTo2Pi(theta1); angle_pub_1.publish(...);  (joint2 too)
    void publishJointAngles(double theta1, double theta2) {
        std_msgs::Float64 ros_theta1, ros_theta2;
        ros_theta1.data = mapTo2Pi(theta1);
        ros_theta2.data = mapTo2Pi(theta2);
        angle_pub_1_.publish(ros_theta1);
        angle_pub_2_.publish(ros_theta2);
    }

    // Publish /albc/status. The 11 fields are assembled in the EXACT former order
    // and units (RAD2DEG on the four attitude angles; raw x/y and angular vels).
    void publishStatus(double target_roll, double current_roll,
                       double target_pitch, double current_pitch,
                       double target_x, double target_y,
                       double current_x, double current_y,
                       double angular_vel_roll, double angular_vel_pitch,
                       double angular_vel_yaw) {
        std_msgs::Float64MultiArray status_msg;
        status_msg.data = {
            RAD2DEG(target_roll), RAD2DEG(current_roll),
            RAD2DEG(target_pitch), RAD2DEG(current_pitch),
            target_x, target_y,
            current_x, current_y,
            angular_vel_roll, angular_vel_pitch, angular_vel_yaw
        };
        status_pub_.publish(status_msg);
    }

private:
    ros::Publisher angle_pub_1_;
    ros::Publisher angle_pub_2_;
    ros::Publisher status_pub_;
};

}  // namespace albc

#endif  // ALBC_CONTROL_STATUS_PUBLISHER_H
