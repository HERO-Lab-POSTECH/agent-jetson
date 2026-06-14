// dvl_position.cpp — DVL 콜백 4개 본문 (agent.ino에서 cut-paste, 산술 글자단위 보존)
// control_T 0/1/3/4 Tx/Ty 식·X/Y 적분·error 계산·save_acc 버퍼 시프트·Th 믹싱·clamp(±100) 전부 원본 그대로.
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

void msgCallback_cont_para(const hero_msgs::hero_agent_cont_para &msg)
{
  control_T = msg.control_T;

  Kp = msg.Kp;
  Ki = msg.Ki;
  Kd = msg.Kd;

  Mb = msg.Mb;
  KKp = msg.KKp;
  KKv = msg.KKv;
}

void msgCallback_dvl_velocity(const hero_msgs::hero_agent_dvl_velocity &msg)
{

  double time_gap = (double)msg.TIME / 1000.0;

  if (msg.VALID == 'y')
  {

    X += (msg.VX * time_gap) * cos(yaw) + (msg.VY * time_gap) * sin(yaw); //
    Y += (msg.VX * time_gap) * sin(yaw) + (msg.VY * time_gap) * cos(yaw); //

    error_x = TARGET_X - X;
    error_y = TARGET_Y - Y;

    error_d_x = (error_x - error_x_pre) / time_gap;
    error_d_y = (error_y - error_y_pre) / time_gap;
    if (error_d_x > 1 || error_d_x < -1)
      error_d_x = 0;

    error_sum_x += error_x;
    error_sum_y += error_y;

    a_x = (error_d_x - error_d_x_pre) / time_gap;
    a_y = (error_d_y - error_d_y_pre) / time_gap;
    if (a_x > 1 || a_x < -1)
      a_x = 0;

    save_acc_x = -save_acc_x_pre[print_i] / (float)save_acc_count[print_i];

    save_v_x = a_x;
    for (int i = 0; i < 19; i++)
    {
      save_acc_x_pre[i] = save_acc_x_pre[i + 1];
      save_acc_count[i] = save_acc_count[i + 1];
    }
    save_acc_x_pre[19] = acc_x;
    save_acc_count[19] = acc_count;
    acc_x = 0;
    acc_count = 0;

    if (control_T == 0)
    {
      Tx = Kp * error_x + Ki * error_sum_x + Kd * error_d_x;
      Ty = Kp * error_y + Ki * error_sum_y + Kd * error_d_y;
    }
    else if (control_T == 1)
    {
      Tx += Mb * (-pre_a_x + KKv * error_d_x + KKp * error_x);
      Ty += Mb * (-pre_a_y + KKv * error_d_y + KKp * error_y);
    }
    else if (control_T == 3)
    {
      Tx += Mb * (-pre_a_x + KKv * error_d_x + KKp * error_x);
      Ty = Kp * error_y + Ki * error_sum_y + Kd * error_d_y;
    }
    else if (control_T == 4)
    {
      Tx += Mb * (-pre_save_acc_x + KKv * error_d_x + KKp * error_x);
      Ty = Kp * error_y + Ki * error_sum_y + Kd * error_d_y;
    }

    if (Tx > 100)
    {
      Tx = 100;
    }
    else if (Tx < -100)
    {
      Tx = -100;
    }

    if (Ty > 100)
    {
      Ty = 100;
    }
    else if (Ty < -100)
    {
      Ty = -100;
    }

    error_d_x_pre = error_d_x;
    error_d_y_pre = error_d_y;

    pre_v_x = msg.VX;
    pre_v_y = msg.VY;

    pre_save_acc_x = save_acc_x;

    pre_a_x = a_x;
    pre_a_y = a_y;

    error_x_pre = error_x;
    error_y_pre = error_y;

    Th_0 = -Tx + Ty;
    Th_1 = -Tx - Ty;
    Th_2 = Tx - Ty;
    Th_3 = Tx + Ty;

    desired_angle_depth = temp_depth + TARGET_Z;
  }
  else
  {
    Th_0 = 0;
    Th_1 = 0;
    Th_2 = 0;
    Th_3 = 0;
    desired_angle_depth = temp_depth + TARGET_Z;
  }
}

void msgCallback_cont_xy_darknet(const hero_msgs::hero_agent_cont_xy &msg)
{
  darknet_Th_0 = msg.T0;
  darknet_Th_1 = msg.T1;
  darknet_Th_2 = msg.T2;
  darknet_Th_3 = msg.T3;
}
