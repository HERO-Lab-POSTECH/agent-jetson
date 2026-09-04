// pid.h — yaw·depth PID 제어 모듈 (agent.ino에서 분리, 산술 글자단위 보존)
// 보드 avr-g++ = C++98 (__cplusplus=199711L) → constexpr/auto/range-for 금지, static const만.
//
// PID 게인·상태 전역의 *정의*는 agent.ino에 있고, 여기서는 extern 선언만 제공한다.
// PID_control_yaw()는 본문에서 esc_input(thrusters)을 호출하고 yaw/acc_yaw(ahrs)를 읽으므로
// thrusters.h·ahrs.h를 include한다. pwm_m*·yaw·acc_yaw는 그 헤더들이 이미 extern하므로 재선언 안 함.
// 의존: pid → {thrusters, ahrs, config}. thrusters/ahrs는 pid를 include 안 함(사이클 없음).
#ifndef AGENT_PID_H
#define AGENT_PID_H
#include "config.h"
#include "thrusters.h"  // esc_input(), pwm_m* (extern은 여기서 제공)
#include "ahrs.h"       // yaw, acc_yaw (extern은 여기서 제공)

// depth 센서 측정값 (정의는 agent.ino, PID_control_depth가 읽음)
extern double depth;

// ── yaw PID 게인·상태 (정의는 agent.ino, 여기선 extern) ──
extern volatile uint8_t cont_yaw_on;
extern volatile double T;
extern volatile int throttle;
extern volatile double P_angle_gain_yaw;
extern volatile double P_gain_yaw;
extern volatile double I_gain_yaw;
extern volatile double D_gain_yaw;
extern volatile double error_yaw;
extern volatile double error_pid_yaw, error_pid_yaw1;
extern volatile double P_angle_pid_yaw;
extern volatile double P_yaw, I_yaw, D_yaw, PID_yaw;
extern volatile double desired_angle_yaw;

// ── depth PID 게인·상태 (정의는 agent.ino, 여기선 extern) ──
extern volatile uint8_t cont_depth_on;
extern volatile double T_depth;
extern double P_gain_depth;
extern double I_gain_depth;
extern double D_gain_depth;
extern double error_pid_depth, error_pid_depth1;
extern double P_angle_pid_depth;
extern double P_depth, I_depth, D_depth, PID_depth;
extern double desired_angle_depth;

// ── cont_direc 믹싱 입력 (정의는 agent.ino, 여기선 extern) ──
extern volatile uint8_t cont_direc;
extern volatile int move_speed;

void PID_control_yaw();
void PID_control_depth();
#endif // AGENT_PID_H
