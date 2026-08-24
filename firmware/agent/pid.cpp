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

  // ---- 2026-08-24: m4 영구 배제 + 3채널 재배분 -------------------------------
  // m4(7.5시)는 3상 중 한 상이 개방·간헐이고 커넥터에 물리적으로 접근할 수 없다
  // (운영자 확인 2026-08-24). 그래서 이 함수는 m1·m2·m5 세 채널만 쓴다.
  //
  // 세 채널만으로도 (Fx, Fy, Mz) 랭크는 3 이라 방위는 정확히 복원된다. 각 모드가
  // 살아 있는 셋 중 둘만 쓰면 되고, 대가는 방위 오차가 아니라 권한 50% 다:
  //
  //   전/후진 (cont_direc 1·2)  m2 를 뺀다  -> m1·m5    (6시/0시 정확)
  //   좌/우   (cont_direc 3·4)  m5 를 뺀다  -> m1·m2    (3시/9시 정확)
  //   yaw                       m1 을 뺀다  -> m2·m5    (Fx=Fy=0, 순수 yaw)
  //
  // 어느 채널을 빼는지는 배치 기하라서 실물 부호계 가설과 **무관하다** — 부호
  // 가설 8가지 전부에서 같은 채널이 빠진다. 바뀌는 것은 남는 두 계수의 부호뿐이고
  // 여기서는 기존 규약을 그대로 뒀다. 수조에서 어느 축이 반대로 가면 그 분기의
  // 두 계수 부호만 뒤집어라 -- 축당 시험 한 번으로 갈린다.
  // 유도: code/classic_allocation_analysis.py --m4-dead (배포 TAM 에서 재유도)
  //
  // throttle 이 사라진 것도 같은 이유다. 네 채널일 때 throttle 은 순 렌치가 정확히
  // 0 인 널스페이스 항이었는데, m4 가 빠지면 널스페이스가 사라져(3x3 정칙)
  // 4.5시로 |F|=1.0 을 미는 순수 외란이 된다. 3채널에서는 0 말고 실현이 없다.
  // (기본값 40 이 ESC 불감대 +-45 안쪽이라 그동안 증상이 가려져 있었다.)
  pwm_m4 = ESC_NEUTRAL; // 배제 -- 명령 없음

  if (cont_direc == 0) // stop
  {
    pwm_m1 = ESC_NEUTRAL;
    pwm_m2 = -PID_yaw + ESC_NEUTRAL;
    pwm_m5 = -PID_yaw + ESC_NEUTRAL;
  }
  else if (cont_direc == 1) // backward
  {
    pwm_m1 = ESC_NEUTRAL - move_speed;
    pwm_m2 = -PID_yaw + ESC_NEUTRAL;
    pwm_m5 = -PID_yaw + ESC_NEUTRAL - move_speed;
  }
  else if (cont_direc == 2) // forward
  {
    pwm_m1 = ESC_NEUTRAL + move_speed;
    pwm_m2 = -PID_yaw + ESC_NEUTRAL;
    pwm_m5 = -PID_yaw + ESC_NEUTRAL + move_speed;
  }
  else if (cont_direc == 3) // right
  {
    pwm_m1 = ESC_NEUTRAL - move_speed;
    pwm_m2 = -PID_yaw + ESC_NEUTRAL - move_speed;
    pwm_m5 = -PID_yaw + ESC_NEUTRAL;
  }
  else if (cont_direc == 4) // left
  {
    pwm_m1 = ESC_NEUTRAL + move_speed;
    pwm_m2 = -PID_yaw + ESC_NEUTRAL + move_speed;
    pwm_m5 = -PID_yaw + ESC_NEUTRAL;
  }

  pwm_m1 = constrain(pwm_m1, ESC_MIN, ESC_MAX);
  pwm_m2 = constrain(pwm_m2, ESC_MIN, ESC_MAX);
  pwm_m4 = constrain(pwm_m4, ESC_MIN, ESC_MAX);
  pwm_m5 = constrain(pwm_m5, ESC_MIN, ESC_MAX);

  // 2026-08-12: esc_input() 2줄을 여기서 agent.ino 의 loop() 로 옮겼다.
  // 이 함수가 유일한 전송자였기 때문에 DEPTH 단독 제어가 죽어 있었다 —
  // PID_control_depth() 는 pwm_m0/pwm_m3 를 계산만 하고 자기 전송이 없어서,
  // yaw 가 꺼져 있으면 깊이 출력이 계산된 뒤 그대로 버려졌고 yaw 가 켜져 있을
  // 때만 이 프레임에 얹혀 나갔다(수조 실측 2026-08-12). 이제 두 PID 는 각자
  // 자기 채널만 계산하고 전송은 호출부가 한다.
  //   yaw -> m1, m2, m4, m5      depth -> m0, m3

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
