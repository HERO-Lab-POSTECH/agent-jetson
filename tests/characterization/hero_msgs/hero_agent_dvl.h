// hero_agent_dvl.h — HOST SHIM, tests only. Not the rosserial-generated header.
//
// The real header is produced on the board by regen_ros_lib.sh from
// robot/hero_msgs/msg/hero_agent_dvl.msg, so a host compiler has nothing to
// include. This shim declares the same four fields with the same types, which is
// all firmware/agent/dvl_position.cpp reads, so the file can be compiled here.
//
// It lives beside Arduino.h for the same reason that shim does: firmware/ must
// stay exactly what the board compiles. If the .msg gains a field this shim does
// not, the compile test simply does not exercise it -- it never diverges
// silently, because a field the code reads and the shim lacks fails to compile.
#ifndef AGENT_TEST_HERO_AGENT_DVL_SHIM_H
#define AGENT_TEST_HERO_AGENT_DVL_SHIM_H

namespace hero_msgs {
struct hero_agent_dvl {
    float TARGET_X;   // float32 TARGET_X
    float TARGET_Y;   // float32 TARGET_Y
    float TARGET_Z;   // float32 TARGET_Z
    signed char command;  // int8 command
};
}  // namespace hero_msgs

#endif  // AGENT_TEST_HERO_AGENT_DVL_SHIM_H
