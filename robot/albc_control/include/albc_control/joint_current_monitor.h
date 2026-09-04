// Production header: JointCurrentMonitor — caches the latest joint motor
// currents for the dashboard readout.
//
// Absorbs the former globals joint_current1_mA / joint_current2_mA and the free
// function jointCurrentsCallback (albc_controller.cpp): onJointCurrents() is the
// byte-identical subscriber callback (guard size>=2, store [0]/[1]); the getters
// feed the dashboard's "Motor J1/J2 mA" line. Only "where the state lives"
// changes (file-scope globals -> class members); never the values or the guard.
//
// ROS-dependent: the callback takes std_msgs::Float32MultiArray, so it includes
// that message header. ros/ros.h is not needed (no NodeHandle/publisher here).

#ifndef ALBC_CONTROL_JOINT_CURRENT_MONITOR_H
#define ALBC_CONTROL_JOINT_CURRENT_MONITOR_H

#include <std_msgs/Float32MultiArray.h>

namespace albc {

class JointCurrentMonitor {
public:
    // /albc/joint_currents subscriber callback. Byte-identical to the former
    // jointCurrentsCallback: only updates when at least two values are present.
    void onJointCurrents(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() >= 2) {
            joint_current1_mA_ = msg->data[0];
            joint_current2_mA_ = msg->data[1];
        }
    }

    // Latest cached currents [mA] for the dashboard.
    float joint1() const { return joint_current1_mA_; }
    float joint2() const { return joint_current2_mA_; }

private:
    float joint_current1_mA_ = 0.0f;
    float joint_current2_mA_ = 0.0f;
};

}  // namespace albc

#endif  // ALBC_CONTROL_JOINT_CURRENT_MONITOR_H
