// dvl_position.h — DVL XY 적분·control_T 위치제어 믹싱 모듈 (agent.ino에서 분리, 산술 글자단위 보존)
// 보드 avr-g++ = C++98 (__cplusplus=199711L) → constexpr/auto/range-for 금지, static const만.
//
// DVL 전역의 *정의*는 agent.ino에 있고, 여기서는 extern 선언만 제공한다.
// 콜백 본문은 yaw(ahrs)·depth/desired_angle_depth/Th_*/darknet_Th_*(pid)·acc_x/acc_count(ahrs)를
// 읽고/쓰므로 pid.h를 include한다(pid → {thrusters,ahrs,config}; pid는 dvl을 include 안 함 → 사이클 없음).
// 따라서 yaw·acc_x·acc_count(ahrs.h), depth·desired_angle_depth·Th_*·darknet_Th_*(pid.h)는 재선언하지 않는다.
#ifndef AGENT_DVL_POSITION_H
#define AGENT_DVL_POSITION_H
#include "config.h"
#include "pid.h"   // depth, desired_angle_depth, Th_*, darknet_Th_* (extern) + ahrs.h 경유 yaw·acc_x·acc_count

// 콜백 시그니처용 ROS 메시지 헤더
#include "hero_msgs/hero_agent_dvl.h"
#include "hero_msgs/hero_agent_dvl_velocity.h"
#include "hero_msgs/hero_agent_cont_xy.h"
#include "hero_msgs/hero_agent_cont_para.h"

// ── DVL 적분·제어 전역 (정의는 agent.ino, 여기선 extern) ──
extern volatile double X, Y, Z;
extern volatile double Tx, Ty;
extern volatile double error_sum_x, error_sum_y;

extern volatile double Kp, Ki, Kd;
extern volatile double Mb, KKp, KKv;

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

extern volatile float save_acc_x;
extern volatile float pre_save_acc_x;
extern volatile float save_v_x;
extern volatile float save_acc_x_pre[20];
extern volatile int save_acc_count[20];
extern volatile int print_i;

// ── DVL 콜백 4개 ──
void msgCallback_dvl(const hero_msgs::hero_agent_dvl &msg);
void msgCallback_dvl_velocity(const hero_msgs::hero_agent_dvl_velocity &msg);
void msgCallback_cont_para(const hero_msgs::hero_agent_cont_para &msg);
void msgCallback_cont_xy_darknet(const hero_msgs::hero_agent_cont_xy &msg);
#endif // AGENT_DVL_POSITION_H
