#ifndef HERO_MSGS_TOPICS_H
#define HERO_MSGS_TOPICS_H

// ROS topic names shared by the Jetson-side packages.
namespace hero_msgs {
namespace topics {

static const char* const SENSORS        = "/hero_agent/sensors";
static const char* const STATE          = "/hero_agent/state";
static const char* const COMMAND        = "/hero_agent/command";
static const char* const KEY_INPUT      = "/hero_agent/key_input";
static const char* const DVL            = "/hero_agent/dvl";
static const char* const THRUSTER_PWM   = "/hero_agent/thruster_pwm";
static const char* const JOINT_STATES   = "/albc/joint_states";
static const char* const JOINT_CURRENTS = "/albc/joint_currents";
static const char* const JOINT_GUARD    = "/albc/joint_guard";
static const char* const ALBC_STATUS    = "/albc/status";
static const char* const THRUSTER_CMD   = "/albc/thruster_cmd";
static const char* const JOINT1_CMD     = "/albc/joint1_cmd";
static const char* const JOINT2_CMD     = "/albc/joint2_cmd";
static const char* const RL_COMMAND     = "/albc/rl_command";

}  // namespace topics
}  // namespace hero_msgs

#endif  // HERO_MSGS_TOPICS_H
