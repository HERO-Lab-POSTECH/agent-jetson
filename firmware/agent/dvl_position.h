// dvl_position.h — DVL XY 적분·control_T 위치제어 믹싱 모듈 (agent.ino에서 분리, 산술 글자단위 보존)
// 보드 avr-g++ = C++98 (__cplusplus=199711L) → constexpr/auto/range-for 금지, static const만.
//
// DVL 전역의 *정의*는 agent.ino에 있고, 여기서는 extern 선언만 제공한다.
// 2026-09-04: 발행자가 없는 콜백 3개(dvl_velocity·cont_para·cont_xy_darknet)와 그것들만
// 쓰던 전역을 제거했다. 남은 것은 msgCallback_dvl 하나 — /hero_agent/dvl 의 리셋·목표 저장.
// depth(pid.h)를 읽으므로 pid.h는 계속 include한다(pid → {thrusters,ahrs,config}; 사이클 없음).
//
// !! 열린 회로: TARGET_X/Y/Z 를 읽는 코드가 이제 없다. 삭제된 msgCallback_dvl_velocity 가
//    desired_angle_depth = temp_depth + TARGET_Z 를 하던 유일한 자리였고, 그 토픽에는
//    2022년 이후 발행자가 없었다. 즉 teleop 의 r/f heave 키는 삭제 전에도 펌웨어 깊이
//    목표를 움직이지 못했다(살아 있는 경로는 펌웨어 키 'o'/'l' → desired_angle_depth +-0.1).
//    이 경로를 되살릴지 없앨지는 로봇 앞에서 정할 결정이라 그대로 두었다.
#ifndef AGENT_DVL_POSITION_H
#define AGENT_DVL_POSITION_H
#include "config.h"
#include "pid.h"   // depth, desired_angle_depth (extern) + ahrs.h 경유 yaw·acc_x·acc_count

// 콜백 시그니처용 ROS 메시지 헤더
#include "hero_msgs/hero_agent_dvl.h"

// ── DVL 적분·제어 전역 (정의는 agent.ino, 여기선 extern) ──
extern volatile double X, Y, Z;
extern volatile double Tx, Ty;
extern volatile double error_sum_x, error_sum_y;

extern int control_T;

extern volatile double TARGET_X, TARGET_Y, TARGET_Z;
extern volatile double temp_depth;

extern volatile double error_x, error_y;
extern volatile double error_d_x, error_d_y;
extern volatile double error_x_pre, error_y_pre;
extern volatile double a_x, a_y;
extern volatile double error_d_x_pre, error_d_y_pre;
extern volatile double pre_a_x, pre_a_y;
extern volatile double pre_v_x, pre_v_y;

// ── DVL 콜백 ──
void msgCallback_dvl(const hero_msgs::hero_agent_dvl &msg);
#endif // AGENT_DVL_POSITION_H
