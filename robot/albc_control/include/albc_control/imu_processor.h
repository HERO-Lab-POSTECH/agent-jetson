// Production header: ImuProcessor — IMU callback wrapper for albc_controller.
//
// Encapsulates the former global imuCallback (albc_controller.cpp:153-162,
// originally :205-214). onImu() delegates the rotation math to the Task-1
// promoted oracle rotateImu() in imu_rotation.h — the byte-identical
// transcription of the original inline rotation. Only "where the result is
// stored" changes (controller cached state -> class members); never the math.
//
// Sign/operation conventions live in imu_rotation.h (PITCH negation, yaw
// pass-through) and are NOT duplicated here — this class is plumbing only.
//
// NOTE: this header is NOT ROS-free. As the IMU callback it must take the
// hero_msgs::hero_agent_sensor message, so it includes that message header.
// ros/ros.h is not needed (no NodeHandle / publisher here) and is omitted.

#ifndef ALBC_CONTROL_IMU_PROCESSOR_H
#define ALBC_CONTROL_IMU_PROCESSOR_H

#include "hero_msgs/hero_agent_sensor.h"
#include "albc_control/imu_rotation.h"  // rotateImu, ImuRpy

namespace albc {

class ImuProcessor {
public:
    // Update the yaw mounting offset [rad]. Mirrors the former global
    // imu_yaw_offset_rad write (main param load + reconfigureCallback).
    void setOffset(double rad) { imu_yaw_offset_rad_ = rad; }

    // IMU subscriber callback. Byte-identical to the former imuCallback:
    // delegates to rotateImu(ROLL, PITCH, YAW, offset) and caches the result.
    void onImu(const hero_msgs::hero_agent_sensor::ConstPtr& msg) {
        ImuRpy out = rotateImu(msg->ROLL, msg->PITCH, msg->YAW, imu_yaw_offset_rad_);
        current_roll_  = out.roll;
        current_pitch_ = out.pitch;
        current_yaw_   = out.yaw;
    }

    // Cached body-frame attitude [rad] from the latest IMU message.
    double roll()  const { return current_roll_; }
    double pitch() const { return current_pitch_; }
    double yaw()   const { return current_yaw_; }

private:
    double current_roll_      = 0.0;
    double current_pitch_     = 0.0;
    double current_yaw_       = 0.0;
    double imu_yaw_offset_rad_ = 0.0;
};

}  // namespace albc

#endif  // ALBC_CONTROL_IMU_PROCESSOR_H
