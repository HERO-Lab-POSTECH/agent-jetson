#include <avr/interrupt.h>

#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include "MS5837.h"

#include <ros.h>
#include <std_msgs/Int8.h>
#include "hero_msgs/hero_agent_sensor.h"
#include "hero_msgs/hero_agent_state.h"

#include "hero_msgs/hero_agent_dvl.h"

// RL thruster mixer command (float32[6] thrust in [-1,1], ch T0..T5 = m0..m5).
#include "hero_msgs/hero_agent_thruster_cmd.h"

// for PIN SETTING---------------------
#define F_CPU 16000000UL

#include "config.h"
#include "ahrs.h"
#include "thrusters.h"
#include "pid.h"
#include "dvl_position.h"
#include "io.h"
//--------------------------------------

// 메인 스케치 자체 함수 forward 선언 (정의가 호출보다 뒤 — Arduino auto-prototype 대체)
void Initialization(void);

// for ROS serial libaray for PUBLISH---------------------
ros::NodeHandle nh; // main handle

hero_msgs::hero_agent_state state_msg;            // state
hero_msgs::hero_agent_sensor sensors_msg;         // sensors

ros::Publisher pub_state(TOPIC_STATE, &state_msg);
ros::Publisher pub_sensors(TOPIC_SENSORS, &sensors_msg);

//--------------------------------------

// for DEPTH sensor---------------------

MS5837 DEPTH_Sensor;

double depth;
//--------------------------------------

// for AHRS_3DM_GX5----------------------
volatile int serial_count = 0;
volatile uint8_t inputString[100];       // a String to hold incoming data
volatile boolean stringComplete = false; // whether the string is complete

volatile uint8_t imu_check = 0;

// u_data union 정의는 ahrs.cpp로 이동 (파싱 전용, ISR은 미사용).

volatile float roll, pitch, yaw;
volatile float yaw_temp;
volatile float acc_roll, acc_pitch, acc_yaw;
volatile float acc_x = 0, acc_y = 0, acc_z = 0;
volatile int acc_count = 0;
volatile uint8_t ahrs_valid1, ahrs_valid2, ahrs_valid3;

volatile uint8_t yaw_calid_command = 0;
volatile char yaw_calib = 0;
volatile float pre_yaw = 0;
volatile float yaw_calib2 = 0;
//--------------------------------------

// for Trusters-------------------------
volatile int pwm_m0 = ESC_NEUTRAL, pid_pwm_m0;
volatile int pwm_m1 = ESC_NEUTRAL, pid_pwm_m1;
volatile int pwm_m2 = ESC_NEUTRAL, pid_pwm_m2;
volatile int pwm_m3 = ESC_NEUTRAL, pid_pwm_m3;
volatile int pwm_m4 = ESC_NEUTRAL, pid_pwm_m4;
volatile int pwm_m5 = ESC_NEUTRAL, pid_pwm_m5;
//--------------------------------------

// for RL thruster mixer ----------------
// PWM = ESC_NEUTRAL + action * SPAN, then constrain (BEFORE esc_input — the
// uint16 esc_input param wraps a negative int to ~65000 otherwise, mirroring
// the PID path's constrain-before-esc_input at pid.cpp:56-62).
// calibration knob: SPAN·중립점은 실추력 측정 후 튜닝. 2026-08-12 부터 6채널이
// 동일 SPAN·bias 0 이다 — sim 은 6개 스러스터에 같은 max_thrust(50.0)를 주는데
// 펌웨어만 수직에 절반을 줬던 부채를 갚은 것. 이전 값(수직 ±150 + DEPTH_BIAS 30)
// 이 만든 두 결함을 건식 실측으로 확인하고 제거했다:
//   (1) 크리핑 — RL 명령 0 일 때 수직이 1470 에 앉는데 ESC 불감대 하단이 1450
//       이라 여유가 20us. 전 채널 0 을 발행해도 m0 가 계속 움찔거렸다(실측).
//       정책의 "추력 0"이 실기에선 0 이 아니었다 = 학습 분포에 없는 plant 편향.
//   (2) 비대칭·권한 손실 — 중립 1470 기준 양은 a>=0.50, 음은 a<=-0.13 에서야
//       불감대를 벗어나 양방향 권한의 절반이 죽었다.
// ESC 불감대 실측(2026-08-12, 건식): 상단 1545 / 하단 약 1450, 반폭 약 48us.
// span 300·bias 0 이면 정규화 불감대가 6채널 모두 0.15 로 같아져 믹서에서
// 보상식 하나로 처리된다. 불감대 보상 자체는 thruster_mixer.py 가 소유한다.
static const int RL_PWM_SPAN_HORZ = 300; // 전 채널: 1500 +- 300 = ESC_MIN/MAX
// B2: inter-message watchdog. mixer 크래시/rosserial 끊김 시 firmware가 마지막
// PWM을 영원히 latch하지 않게, RL 메시지가 이 시간(ms) 넘게 안 오면 전 채널
// NEUTRAL. RL 콜백과 relay_on()이 last_rl_msg_ms를 갱신한다 — relay 토글 직후
// (RL 메시지가 아직 없을 때) loop가 즉시 오발동 NEUTRAL 내지 않게. (relay_on()의
// 옛 5초 블로킹은 제거됨 — io.cpp 참조.)
static const unsigned long RL_TIMEOUT_MS = 300;
volatile uint8_t rl_active = 0;             // 1 = at least one RL msg seen (arms the watchdog)
volatile unsigned long last_rl_msg_ms = 0;  // millis() of last RL msg (and relay_on refresh)
//--------------------------------------

// for Control yaw and depth---------------------
volatile uint8_t cont_yaw_on = 0;
volatile uint8_t cont_depth_on = 0;

// TODO(4번 flash 조각): T/T_depth를 측정 loop_speed 기반 동적 dt로.
//   현재는 고정 명목값(T=0.004는 실제 루프주기 ~9ms와도 불일치).
//   변경 시 D/I 게인 재튜닝 필수(거동 변경) → flash 검증과 함께 opt-in.
volatile double T = 0.004;      // Loop time.
volatile double T_depth = 0.04; // Loop time.

volatile int throttle = 40;

volatile double P_angle_gain_yaw = 6.5;
volatile double P_gain_yaw = 10;
volatile double I_gain_yaw = 1;
volatile double D_gain_yaw = 0.5;

double P_gain_depth = 1100.0;
double I_gain_depth = 10.0;
double D_gain_depth = 100.0;

volatile double error_yaw;
volatile double error_pid_yaw, error_pid_yaw1;
volatile double P_angle_pid_yaw;
volatile double P_yaw, I_yaw, D_yaw, PID_yaw;
// 2026-08-24: 부팅 기본값 1 rad(57.3°) -> 0. 이건 pid.cpp 의 불감대 FF 보다 **먼저**
// 들어가야 하는 선결 항목이다. 지금까지는 불감대가 이 명령을 삼키고 있었을 뿐이고
// (2026-08-24 세션 503 s 동안 Target_yaw 가 +1.00 이었다가 운영자가 'Z' 를 눌러서야
// 0 이 됐다), FF 가 들어가면 전원 투입 + 'Y' 즉시 로봇이 57° 를 향해 돈다.
// 분석: notes/2026-08-24-fault-tolerant-allocation-analysis.md §3-6 "부팅 setpoint"
volatile double desired_angle_yaw = 0;

double error_pid_depth, error_pid_depth1;
double P_angle_pid_depth;
double P_depth, I_depth, D_depth, PID_depth;
double desired_angle_depth = 0.5;
//--------------------------------------

volatile uint8_t cont_direc = 0;
volatile int move_speed = 20;

// for STATE addition
volatile uint8_t cont_Yaw = 0;
volatile uint8_t cont_Depth = 0;
volatile uint8_t state_Relay = 0;
volatile uint8_t state_Laser = 0;

volatile int State_all = 0;
//-------------------------------------

volatile double X = 0, Y = 0, Z = 0;
volatile double Tx = 0, Ty = 0;
volatile double error_sum_x = 0, error_sum_y = 0;

int control_T = 0;

volatile double TARGET_X = 0, TARGET_Y = 0, TARGET_Z = 0;
volatile double temp_depth = 0;

volatile double error_x = 0, error_y = 0;
volatile double error_d_x = 0, error_d_y = 0;
volatile double error_x_pre = 0, error_y_pre = 0;
volatile double a_x = 0, a_y = 0;
volatile double error_d_x_pre = 0, error_d_y_pre = 0;
volatile double pre_a_x = 0, pre_a_y = 0;
volatile double pre_v_x = 0, pre_v_y = 0;

// msgCallback_dvl() 정의는 dvl_position.cpp (선언은 dvl_position.h).
// DVL 적분·제어 전역의 *정의*는 agent.ino에 그대로 유지(dvl_position.h가 extern).


//---------------------------------------

int test_cont_set = 0;
unsigned long time_init;
void messageCommand(const std_msgs::Int8 &command_msg)
{
  int Command = command_msg.data;

  // --- Toggles (self-toggle: 한 번 누르면 켜지고 또 누르면 꺼짐) ---
  if (Command == 'R') // Relay
  {
    if (state_Relay) relay_off(); else relay_on();
  }
  else if (Command == 'Y') // Yaw control
  {
    cont_yaw_on = !cont_yaw_on;
    cont_Yaw = cont_yaw_on;
    // 2026-08-24: 상승엣지에서 적분기 리셋. I_yaw 는 그동안 **어디서도** 리셋되지
    // 않아서(pid.cpp 의 클램프가 유일한 언급) 부팅 이후 누적분이 영원히 남았다.
    // 불감대 FF 전에는 적분기가 출력에 닿지 않아 무증상이었다. 자세한 근거:
    // notes/2026-08-24-fault-tolerant-allocation-analysis.md §3-6 "2순위 — 적분기"
    if (cont_yaw_on) I_yaw = 0;
    // Falling edge with depth already off: flush NEUTRAL, see the 'D' handler.
    // Skipped while RL owns the ESCs -- messageThruster() drives them directly
    // and a stray frame here would fight it for one cycle.
    else if (!cont_depth_on && !rl_active) {
      pwm_m1 = ESC_NEUTRAL; pwm_m2 = ESC_NEUTRAL;
      pwm_m4 = ESC_NEUTRAL; pwm_m5 = ESC_NEUTRAL;
      esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
      esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);
    }
  }
  else if (Command == 'D') // Depth control
  {
    cont_depth_on = !cont_depth_on;
    cont_Depth = cont_depth_on;
    // Same rising-edge convention as yaw above (2026-08-24). Without it a
    // previously saturated I_depth is re-applied the instant depth control is
    // switched back on.
    if (cont_depth_on) I_depth = 0;
    // Falling edge: send one NEUTRAL frame for the channels this controller
    // owned. ESC transmission below only runs while a controller is enabled, so
    // disabling the LAST one used to leave the previous frame latched and the
    // thrusters running at whatever they were.
    else if (!cont_yaw_on && !rl_active) {
      pwm_m0 = ESC_NEUTRAL; pwm_m3 = ESC_NEUTRAL;
      esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
      esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);
    }
  }
  else if (Command == 'L') // Laser
  {
    if (state_Laser) laser_off(); else laser_on();
  }
  // --- One-shot ---
  else if (Command == 'Z') // Yaw reset
  {
    desired_angle_yaw = 0;
    yaw_calid_command = 1;
    // 2026-08-24: 'Z' 는 "yaw 리셋" 키인데 정작 적분기를 안 비웠다. 목표각과 기준각을
    // 둘 다 0 으로 되돌리면서 누적분만 남겨두면 리셋 직후에 그 누적분이 그대로 밀어낸다.
    I_yaw = 0;

    test_cont_set = 0;
  }
  else if (Command == 'P') // PWM neutral (all ESC)
  {

    pwm_m0 = ESC_NEUTRAL;
    pwm_m1 = ESC_NEUTRAL;
    pwm_m2 = ESC_NEUTRAL;
    pwm_m3 = ESC_NEUTRAL;
    pwm_m4 = ESC_NEUTRAL;
    pwm_m5 = ESC_NEUTRAL;

    esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);
  }
  // --- Thruster jog ---
  else if (Command == 'q') // stop
  {
    cont_direc = 0;
  }
  else if (Command == 's') // backward
  {
    cont_direc = 1;
  }
  else if (Command == 'w') // forward
  {
    cont_direc = 2;
  }
  else if (Command == 'd') // right
  {
    cont_direc = 3;
  }
  else if (Command == 'a') // left
  {
    cont_direc = 4;
  }
  // --- Speed / throttle ---
  else if (Command == '+') // move_speed +
  {
    move_speed += 10;
  }
  else if (Command == '-') // move_speed -
  {
    move_speed -= 10;
  }
  else if (Command == 'u') // throttle +
  {
    throttle += 10;
  }
  else if (Command == 'j') // throttle -
  {
    throttle -= 10;
  }
  // --- Setpoint ---
  else if (Command == 'i') // yaw +0.1
  {
    desired_angle_yaw += 0.1;
  }
  else if (Command == 'k') // yaw -0.1
  {
    desired_angle_yaw -= 0.1;
  }
  else if (Command == 'o') // depth +0.1
  {
    desired_angle_depth += 0.1;
  }
  else if (Command == 'l') // depth -0.1
  {
    desired_angle_depth -= 0.1;
  }
  // --- Gripper ---
  else if (Command == 'c')
  {
    gripper_set(GRIPPER_OPEN); // open gripper
  }
  else if (Command == 'v')
  {
    gripper_set(GRIPPER_STOP); // stop gripper
  }
  else if (Command == 'b')
  {
    gripper_set(GRIPPER_CLOSE); // close gripper
  }

  State_all = 0;
  if (cont_Yaw == 1)
  {
    State_all += 1;
  }
  if (cont_Depth == 1)
  {
    State_all += 2;
  }
  if (state_Relay == 1)
  {
    State_all += 4;
  }
  if (state_Laser == 1)
  {
    State_all += 8;
  }
}

ros::Subscriber<std_msgs::Int8> sub_command(TOPIC_COMMAND,
                                            &messageCommand);

// RL thruster: map one action ch in [-1,1] to a constrained ESC PWM.
// CONSTRAIN BEFORE esc_input — esc_input's uint16 param would wrap a negative
// int (out-of-range action) to ~65000 and emit a garbage high-throttle byte.
// ⚠️ 2026-08-12 부터 **호출자 6개 전부 bias 0** 이다 (ESC_MIN/ESC_MAX, span 300).
// 아래 bias 설명은 인자를 살려둔 이유이지 현재 쓰임이 아니다 — 수직이 bias 30 을
// 쓰던 시절의 서술로 읽지 말 것. 왜 뺐는지는 RL_PWM_SPAN_HORZ 주석 참조.
// bias != 0 을 다시 쓸 경우: 중립이 ESC_NEUTRAL-bias 가 되므로 constrain 창도
// 같이 -bias 만큼 옮겨야 한다 (lo-bias, hi-bias). 안 옮기면 창은 ESC_NEUTRAL 에
// 중심을 두는데 명령은 ESC_NEUTRAL-bias 에 중심을 둬서 아래쪽이 clip 되고 위쪽은
// 도달 불가가 된다 — 정책이 학습한 대칭 ±1 대비 권한이 조용히 비대칭이 된다.
static int rl_action_to_pwm(float a, int span, int lo, int hi, int bias)
{
  // NaN fails BOTH comparisons below and would reach the (int) cast and the ESC
  // frame as an arbitrary count. Neutral is the only safe reading of "no number".
  if (a != a) a = 0.0f;
  if (a > 1.0f) a = 1.0f;
  else if (a < -1.0f) a = -1.0f;
  int pwm = ESC_NEUTRAL - bias + (int)(a * span);
  return constrain(pwm, lo - bias, hi - bias);
}

// RL thruster mixer command callback. Six channels in [-1,1], T0..T5 = m0..m5.
// B3: force PID OFF + reset stale teleop bias EVERY message (self-healing against
//     an accidental 'Y'/'D' PID re-activation or a stale throttle/move_speed).
// B2: stamp last_rl_msg_ms so the loop() watchdog can NEUTRAL on link loss.
void messageThruster(const hero_msgs::hero_agent_thruster_cmd &msg)
{
  // B3 — take exclusive ownership of the ESCs away from the legacy PID/teleop.
  cont_yaw_on = 0;
  cont_depth_on = 0;
  cont_Yaw = 0;
  cont_Depth = 0;
  cont_direc = 0;   // no teleop jog mixing
  throttle = 0;     // drop the 40-count stale bias the PID path would add
  move_speed = 0;

  // 6채널 동일 매핑 (2026-08-12). 수직 m0/m3 도 HORZ span·ESC 한계·bias 0 —
  // 위 RL_PWM_SPAN_HORZ 주석의 (1) 크리핑 / (2) 비대칭 참조. bias 0 이라 RL
  // 명령 0 이 정확히 ESC_NEUTRAL(1500) 이고, 이는 B2 워치독의 NEUTRAL 과도 같다.
  pwm_m0 = rl_action_to_pwm(msg.thrust[0], RL_PWM_SPAN_HORZ, ESC_MIN, ESC_MAX, 0);
  pwm_m1 = rl_action_to_pwm(msg.thrust[1], RL_PWM_SPAN_HORZ, ESC_MIN, ESC_MAX, 0);
  pwm_m2 = rl_action_to_pwm(msg.thrust[2], RL_PWM_SPAN_HORZ, ESC_MIN, ESC_MAX, 0);
  pwm_m3 = rl_action_to_pwm(msg.thrust[3], RL_PWM_SPAN_HORZ, ESC_MIN, ESC_MAX, 0);
  pwm_m4 = rl_action_to_pwm(msg.thrust[4], RL_PWM_SPAN_HORZ, ESC_MIN, ESC_MAX, 0);
  pwm_m5 = rl_action_to_pwm(msg.thrust[5], RL_PWM_SPAN_HORZ, ESC_MIN, ESC_MAX, 0);

  esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
  esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);

  // B2 — arm + refresh the watchdog (cleared to NEUTRAL in loop() on timeout).
  rl_active = 1;
  last_rl_msg_ms = millis();
}

ros::Subscriber<hero_msgs::hero_agent_thruster_cmd> sub_thruster(TOPIC_THRUSTER_PWM,
                                                                 &messageThruster);

ros::Subscriber<hero_msgs::hero_agent_dvl> sub_dvl(TOPIC_DVL, &msgCallback_dvl);

//--------------------------------------------
//--------------------------------------------
volatile int check_hz = 0;
// Incoming AHRS data--------------------------------------
ISR(USART1_RX_vect)
{ // USART1에 RX가 들어왔을때 동작하는 interrupt

  char inChar = (char)UDR1;

  if (imu_check == 0 && inChar == (char)MIP_SYNC1)
  {
    imu_check = 1;
    inputString[0] = inChar;
  }
  else if (imu_check == 1 && inChar == (char)MIP_SYNC2)
  {
    imu_check = 2;
    inputString[1] = inChar;
  }
  else if (imu_check == 2 && inChar == (char)MIP_DESC)
  {
    imu_check = 3;
    inputString[2] = inChar;
    serial_count = 3;
  }
  else if (imu_check == 3)
  {
    inputString[serial_count] = inChar;
    serial_count++;
  }
  // if the incoming character is a newline, set a flag so the main loop can
  // do something about it:
  // inputString[19] == 1 &&
  if (serial_count == MIP_PACKET_LEN && inputString[35] == 1) //수신이 완료
  {
    ahrs_parse_packet(); // 패킷 완성 파싱 본문(u_data 추출·yaw unwrap) → ahrs.cpp로 이동
  }
  else if (serial_count > MIP_PACKET_LEN) //데이터가 10개 넘게 들어옴 (데inputString[19]이터 수신 실패)
  {
    inputString[0] = '\0';
    serial_count = 0;
    stringComplete = false;
    imu_check = 0;
  }
}

// UART1_write() 정의는 ahrs.cpp로 이동 (선언은 ahrs.h).
// UART2_write() 정의는 thrusters.cpp로 이동 (선언은 thrusters.h).
//----------------------------------------------------------
unsigned long start_time = 0;
unsigned long current_time = 0;
int loop_count = 0;
int depth_count = 0;
int loop_speed = 0;

void setup()
{
  nh.initNode();

  nh.advertise(pub_state);
  nh.advertise(pub_sensors);
  // /hero_agent/result was un-advertised 2026-08-25 (zero subscribers measured,
  // msg DEPRECATED since 2026-06-14, 724 B/s = 13 % of the 57600-baud link spent
  // on nothing) and the publisher itself is gone 2026-09-04.
  nh.subscribe(sub_command);
  nh.subscribe(sub_thruster);
  // /hero_agent/{cont_xy_darknet,cont_para,dvl_velocity} un-subscribed 2026-09-04:
  // no publisher has existed since the 2022 DVL/darknet work, and their callbacks
  // are deleted. Their topic constants stay in config.h only as wire history.
  nh.subscribe(sub_dvl);
  Initialization(); // init all 1100ms

  start_time = millis();
  loop_count = 0;
}
// loop is for control
void loop()
{
  current_time = millis() - start_time;
  loop_count++;

  if (current_time > 1000)
  {
    start_time = millis();
    loop_speed = loop_count;
    loop_count = 0;
  }

  if (cont_yaw_on == 1)
    PID_control_yaw();

  // --- Classic-path ESC frame (2026-08-12) -------------------------------
  // The transmit used to sit at the end of PID_control_yaw(), which made DEPTH
  // CONTROL ALONE A NO-OP: PID_control_depth() computes pwm_m0/pwm_m3 and has no
  // transmit of its own, so with yaw OFF the depth output was calculated and
  // thrown away, and with yaw ON it only shipped as a side effect of yaw's frame.
  // Observed in the tank 2026-08-12 ("depth only works if yaw is on"). Now each
  // controller owns exactly its own channels and the frame is sent from here, so
  // either one works alone:   yaw -> m1,m2,m4,m5     depth -> m0,m3
  //
  // A controller that is OFF pins its own channels to NEUTRAL instead of latching
  // whatever the other path last wrote (the ESCs hold the last PWM forever, so an
  // unpinned channel would keep spinning after its controller was switched off).
  //
  // Deliberately NOT gated on rl_active: messageThruster() forces BOTH flags to 0
  // on every RL message (B3, see :376-379), so this block is already unreachable
  // while the RL mixer is streaming -- adding a second guard would be dead code.
  //
  // Position: kept HERE (before spinOnce) rather than after the depth state
  // machine so the yaw path keeps its current latency. Depth values therefore
  // ship one loop iteration after they are computed -- which is exactly what
  // already happened whenever both controllers were on.
  if (cont_yaw_on == 1 || cont_depth_on == 1)
  {
    if (cont_yaw_on == 0)
    {
      pwm_m1 = ESC_NEUTRAL;
      pwm_m2 = ESC_NEUTRAL;
      pwm_m4 = ESC_NEUTRAL;
      pwm_m5 = ESC_NEUTRAL;
    }
    if (cont_depth_on == 0)
    {
      pwm_m0 = ESC_NEUTRAL;
      pwm_m3 = ESC_NEUTRAL;
    }
    esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);
  }

  nh.spinOnce();

  // B2 — RL thruster watchdog: once armed, if no RL msg within RL_TIMEOUT_MS the
  // mixer/link is presumed dead → drive all ESCs to NEUTRAL (a latched last-PWM
  // is NOT a stop). relay_on() refreshes last_rl_msg_ms so a relay toggle is not
  // counted as a missed message. millis() wraps at ~49.7 days; the
  // unsigned subtraction stays correct across a single wrap.
  // Concurrency: last_rl_msg_ms is 4 bytes (non-atomic on 8-bit AVR), but the
  // callback runs synchronously INSIDE the nh.spinOnce() above and this read is
  // AFTER spinOnce returns — cooperative, single-threaded, no ISR writes it — so
  // no torn read is possible. volatile is sufficient; do NOT add cli()/sei().
  if (rl_active && (millis() - last_rl_msg_ms > RL_TIMEOUT_MS))
  {
    pwm_m0 = ESC_NEUTRAL;
    pwm_m1 = ESC_NEUTRAL;
    pwm_m2 = ESC_NEUTRAL;
    pwm_m3 = ESC_NEUTRAL;
    pwm_m4 = ESC_NEUTRAL;
    pwm_m5 = ESC_NEUTRAL;
    // NEUTRAL rides the UART2 ESC link, which is INDEPENDENT of the (dead)
    // rosserial PC link — so this frame reaches the ESCs even when the mixer is
    // gone. ESCs latch the last PWM, so one NEUTRAL frame holds them neutral;
    // hence the one-shot (rl_active=0) below. The load-bearing assumption is that
    // this single UART2 frame is delivered (no per-frame retry).
    esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);
    rl_active = 0;  // one-shot NEUTRAL; re-arms on the next RL msg
  }

  // --- Attitude publish, EVERY loop iteration (2026-08-25) ----------------
  // This used to sit inside the depth_count == 3 branch below, so it fired once
  // per FOUR loop iterations -- each carrying its own delay(9) for the MS5837
  // conversion staging. Measured: loop_speed 91 Hz, /hero_agent/sensors 22.6 Hz,
  // exactly 91/4. There is no data dependency justifying that: sensors_msg.DEPTH
  // carries loop_speed, NOT depth, and roll/pitch/yaw/acc_* are written by the
  // USART1 RX ISR (agent.ino:429), which preempts delay() and therefore updates
  // at the AHRS stream rate regardless of where this publish sits.
  //
  // WHY IT MATTERS. The RL policy's constants are all per-TICK -- CONTROL_DT
  // (np_policy.py:65) is hardcoded 0.02, so control_hz 50 is the trained rate.
  // At 50 Hz control against a 22.6 Hz observation, 55 % of ticks re-read the
  // same IMU frame. The 2026-08-25 tank runs bracketed that: control_hz 50 gave
  // a yaw runaway in 13 s, control_hz 20 (fresh frames, but every tick constant
  // 2.5x off) held 22 s then grew a roll oscillation. Neither rate is right;
  // raising the observation rate is what removes the trade.
  // Duplicate-frame rate measured on the bag was 0.2 % at 22.6 Hz, so the AHRS
  // streams well above that -- the gating was here, not in the sensor.
  sensors_msg.ROLL = roll;
  sensors_msg.PITCH = pitch;
  sensors_msg.YAW = yaw;
  sensors_msg.DEPTH = loop_speed;
  sensors_msg.GYRO_X = acc_roll;    // 이미 파싱된 자이로 진값 p (sensor frame)
  sensors_msg.GYRO_Y = acc_pitch;   // q
  sensors_msg.GYRO_Z = acc_yaw;     // r

  pub_sensors.publish(&sensors_msg);

  if (depth_count == 0)
  {
    DEPTH_Sensor.read2();
    depth_count = 1;
    delay(9);
  }
  else if (depth_count == 1)
  {
    depth_count = 2;
    delay(9);
  }
  else if (depth_count == 2)
  {
    DEPTH_Sensor.read4();
    depth = DEPTH_Sensor.depth();

    if (cont_depth_on == 1)
      PID_control_depth();

    depth_count = 3;
    delay(9);
  }
  else if (depth_count == 3)
  {
    depth_count = 0;
    delay(9);

    // DEPTH_Sensor.read();
    // depth = DEPTH_Sensor.depth();

    // sensors_msg moved OUT of this branch (2026-08-25) -- see the block above
    // the depth state machine. The result publisher is gone entirely; both are why.

    state_msg.Yaw = yaw;
    state_msg.Target_yaw = desired_angle_yaw;
    state_msg.Throttle = throttle;
    state_msg.Valid_yaw = inputString[19];
    state_msg.Depth = depth;
    state_msg.Target_depth = desired_angle_depth;
    state_msg.Move_speed = move_speed;
    state_msg.Cont_state = cont_direc;
    state_msg.State_addit = State_all;
    pub_state.publish(&state_msg);
  }
}

// esc_input() 정의는 thrusters.cpp로 이동 (선언은 thrusters.h).

void Initialization(void)
{
  // Serial setting
  // Serial2.begin(115200); //truster control
  UBRR2H = 0x00;
  UBRR2L = 0x08;
  UCSR2A = 0x00;
  UCSR2B = 0x18;
  UCSR2C = 0x06;
  // Serial1 setting (ahrs)
  UBRR1H = 0x00;
  UBRR1L = 0x08;
  UCSR1A = 0x00;
  UCSR1B = 0x98;
  UCSR1C = 0x06;

  // init pressure sensor
  Wire.begin();

  // Init AHRS

  ahrs_init(); // AHRS MIP init 시퀀스(delay 포함) → ahrs.cpp로 이동

  gripper_init();

  // depth sensor init
  while (!DEPTH_Sensor.init())
  {
    delay(5000);
  }
  DEPTH_Sensor.setModel(MS5837::MS5837_30BA);
  DEPTH_Sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  io_init();
}

// PID_control_yaw() / PID_control_depth() 정의는 pid.cpp로 이동 (선언은 pid.h).
// 게인·상태 전역의 *정의*는 위쪽 agent.ino에 그대로 유지(pid.h가 extern).
