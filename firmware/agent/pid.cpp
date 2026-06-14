// pid.cpp — yaw·depth PID 본문 (agent.ino에서 cut-paste, 산술 글자단위 보존)
// P/I/D 식·cont_direc 믹싱·constrain·esc_input 호출 순서 전부 원본 그대로.
// 상수치환(ESC_NEUTRAL/MIN/MAX, DEPTH_*)은 Task2에서 이미 적용된 것을 유지.
#include "pid.h"

void PID_control_yaw()
{
  error_yaw = desired_angle_yaw - yaw;            // angle def
  P_angle_pid_yaw = P_angle_gain_yaw * error_yaw; // angle def + outer P control

  error_pid_yaw = P_angle_pid_yaw - acc_yaw; // Pcontrol_angle - angle rate = PID Goal

  P_yaw = error_pid_yaw * P_gain_yaw;                        // Inner P control
  D_yaw = (error_pid_yaw - error_pid_yaw1) / T * D_gain_yaw; // Inner D control
  I_yaw += (error_pid_yaw)*T * I_gain_yaw;                   // Inner I control
  I_yaw = constrain(I_yaw, -100, 100);                       // I control must be limited to prevent being jerk.

  PID_yaw = P_yaw + D_yaw + I_yaw;

  if (cont_direc == 0) // stop
  {
    pwm_m1 = PID_yaw + throttle + ESC_NEUTRAL;
    pwm_m2 = -PID_yaw + throttle + ESC_NEUTRAL;
    pwm_m4 = -PID_yaw - throttle + ESC_NEUTRAL;
    pwm_m5 = -PID_yaw + throttle + ESC_NEUTRAL;
  }
  else if (cont_direc == 1) // backward
  {
    pwm_m1 = PID_yaw + throttle + ESC_NEUTRAL - move_speed;
    pwm_m2 = -PID_yaw + throttle + ESC_NEUTRAL + move_speed;
    pwm_m4 = -PID_yaw - throttle + ESC_NEUTRAL - move_speed;
    pwm_m5 = -PID_yaw + throttle + ESC_NEUTRAL - move_speed;
  }
  else if (cont_direc == 2) // forward
  {
    pwm_m1 = PID_yaw + throttle + ESC_NEUTRAL + move_speed;
    pwm_m2 = -PID_yaw + throttle + ESC_NEUTRAL - move_speed;
    pwm_m4 = -PID_yaw - throttle + ESC_NEUTRAL + move_speed;
    pwm_m5 = -PID_yaw + throttle + ESC_NEUTRAL + move_speed;
  }
  else if (cont_direc == 3) // right
  {
    pwm_m1 = PID_yaw + throttle + ESC_NEUTRAL - move_speed;
    pwm_m2 = -PID_yaw + throttle + ESC_NEUTRAL - move_speed;
    pwm_m4 = -PID_yaw - throttle + ESC_NEUTRAL - move_speed;
    pwm_m5 = -PID_yaw + throttle + ESC_NEUTRAL + move_speed;
  }
  else if (cont_direc == 4) // left
  {
    pwm_m1 = PID_yaw + throttle + ESC_NEUTRAL + move_speed;
    pwm_m2 = -PID_yaw + throttle + ESC_NEUTRAL + move_speed;
    pwm_m4 = -PID_yaw - throttle + ESC_NEUTRAL + move_speed;
    pwm_m5 = -PID_yaw + throttle + ESC_NEUTRAL - move_speed;
  }

  pwm_m1 = constrain(pwm_m1, ESC_MIN, ESC_MAX);
  pwm_m2 = constrain(pwm_m2, ESC_MIN, ESC_MAX);
  pwm_m4 = constrain(pwm_m4, ESC_MIN, ESC_MAX);
  pwm_m5 = constrain(pwm_m5, ESC_MIN, ESC_MAX);

  esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
  esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);

  error_pid_yaw1 = error_pid_yaw;
}

void PID_control_depth()
{
  error_pid_depth = desired_angle_depth - depth;
  P_depth = error_pid_depth * P_gain_depth;                                // Inner P control
  D_depth = (error_pid_depth - error_pid_depth1) / T_depth * D_gain_depth; // Inner D control
  I_depth += (error_pid_depth)*T_depth * I_gain_depth;                     // Inner I control
  I_depth = constrain(I_depth, -100, 100);                                 // I control must be limited to prevent being jerk.

  PID_depth = P_depth + D_depth + I_depth;

  pwm_m0 = -PID_depth + ESC_NEUTRAL - DEPTH_BIAS;
  pwm_m3 = -PID_depth + ESC_NEUTRAL - DEPTH_BIAS;

  pwm_m0 = constrain(pwm_m0, DEPTH_PWM_MIN, DEPTH_PWM_MAX);
  pwm_m3 = constrain(pwm_m3, DEPTH_PWM_MIN, DEPTH_PWM_MAX);

  error_pid_depth1 = error_pid_depth;
}
