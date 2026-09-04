// dvl_position.cpp — /hero_agent/dvl 콜백 (agent.ino에서 cut-paste, 산술 글자단위 보존)
// 2026-09-04: 발행자가 없던 콜백 3개(dvl_velocity·cont_para·cont_xy_darknet)를 제거했다.
// 그것들이 담고 있던 control_T Tx/Ty 식·X/Y 적분·save_acc 버퍼·Th 믹싱은 함께 사라졌다.
// 복원이 필요하면 커밋 90b13e4 를 보라. 열린 회로 경고는 dvl_position.h 머리말에 있다.
// DVL 전역의 *정의*는 agent.ino, 여기선 dvl_position.h의 extern을 통해 본다.
#include "dvl_position.h"

void msgCallback_dvl(const hero_msgs::hero_agent_dvl &msg)
{
  if (msg.command == 1)
  {
    X = 0;
    Y = 0;
    temp_depth = depth;
    Tx = 0;
    Ty = 0;
    error_sum_x = 0;
    error_sum_y = 0;

    error_x = 0, error_y = 0;
    error_d_x = 0, error_d_y = 0;
    error_x_pre = 0, error_y_pre = 0;
    a_x = 0, a_y = 0;
    error_d_x_pre = 0, error_d_y_pre = 0;
    pre_a_x = 0, pre_a_y = 0;
    pre_v_x = 0, pre_v_y = 0;

    TARGET_X = 0;
    TARGET_Y = 0;
    TARGET_Z = 0;
  }
  else
  {
    TARGET_X = msg.TARGET_X;
    TARGET_Y = msg.TARGET_Y;
    TARGET_Z = msg.TARGET_Z;
  }
}
