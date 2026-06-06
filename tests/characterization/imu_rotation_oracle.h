// Golden oracle: imuCallback IMU yaw-offset rotation (ROS-free extraction)
//
// Byte-identical transcription of albc_controller.cpp:205-214 (imuCallback).
// The ROS plumbing (hero_msgs::hero_agent_sensor::ConstPtr& msg) is dropped:
// the three sensor fields (ROLL/PITCH/YAW) are passed explicitly, and the
// controller member imu_yaw_offset_rad is passed as offset_rad. State writes
// (state.current_roll/pitch/yaw) become struct-return fields. The redesign
// must keep this rotation byte-identical — it changes the robot's behavior.
//
// Sign/operation conventions pinned here (do NOT "fix" without intent):
//   raw_roll  =  ROLL              (no sign change)
//   raw_pitch = -(PITCH)           (PITCH is NEGATED — pinned)
//   c = cos(offset_rad), s = sin(offset_rad)
//   out_roll  =  c*raw_roll + s*raw_pitch
//   out_pitch = -s*raw_roll + c*raw_pitch
//   out_yaw   =  YAW              (passed through, no rotation)

#ifndef IMU_ROTATION_ORACLE_H
#define IMU_ROTATION_ORACLE_H

#include <cmath>   // cos, sin

struct ImuRpy {
    double roll;
    double pitch;
    double yaw;
};

// Pure transcription of albc_controller.cpp:206-213.
// offset_rad is the yaw offset in RADIANS (controller member imu_yaw_offset_rad).
inline ImuRpy rotateImu(double ROLL, double PITCH, double YAW, double offset_rad)
{
    double raw_roll  = ROLL;
    double raw_pitch = -(PITCH);

    // Rotate IMU readings by yaw offset to align with robot body frame
    double c = cos(offset_rad), s = sin(offset_rad);

    ImuRpy out;
    out.roll  =  c * raw_roll + s * raw_pitch;
    out.pitch = -s * raw_roll + c * raw_pitch;
    out.yaw   = YAW;
    return out;
}

#endif // IMU_ROTATION_ORACLE_H
