// config.h — 매직넘버·핀 선언 (값은 원본 agent.ino 리터럴 그대로 보존)
// 보드 Arduino 툴체인이 C++98로 컴파일하므로 constexpr 대신 static const 사용
// (constexpr은 C++11 전용 → 보드에서 컴파일 불가). 생성 코드는 리터럴과 동일.
#ifndef AGENT_CONFIG_H
#define AGENT_CONFIG_H
#include <stdint.h>

// ── ROS topics (wire names are firmware ABI; rename only with a coordinated flash) ──
static const char* const TOPIC_STATE = "/hero_agent/state";
static const char* const TOPIC_SENSORS = "/hero_agent/sensors";
static const char* const TOPIC_COMMAND = "/hero_agent/command";
static const char* const TOPIC_THRUSTER_PWM = "/hero_agent/thruster_pwm";
static const char* const TOPIC_DVL = "/hero_agent/dvl";

// ── 핀 (원본 #define) ──
static const uint8_t PIN_RELAY = 15;
static const uint8_t PIN_GRIPPER_SIG = 20;
static const uint8_t PIN_LED_SIG = 21;
static const uint8_t PIN_RTS = 10;

// ── ESC PWM ──
static const int ESC_NEUTRAL = 1500;
static const int ESC_MIN = 1200;
static const int ESC_MAX = 1800;

// ── yaw 불감대 역보정 (2026-08-24) ──
// ESC 는 1450~1545 us 를 벗어나야 돈다(2026-08-12 건식 실측). 중립 1500 기준 +45 counts.
// 클래식 yaw PID 는 그 아래를 그냥 못 넘겨서, 2026-08-24 수조의 정지 830 s / 16,340 샘플
// 동안 불감대를 **한 번도** 못 벗어났다(최대 |PID_yaw| 44.22 < 45). 즉 yaw 루프가 통째로
// 개루프였다. RL 믹서는 이미 같은 보정을 하고 있다(thruster_mixer.undeadband, D = 0.15).
// 근거·replay 표: notes/2026-08-24-fault-tolerant-allocation-analysis.md §3-2·§3-5·§3-6
static const double YAW_FF_DEADBAND = 45.0;  // = 0.15 * span 300. RL 믹서의 D 와 같은 값
// SLOPE 는 DEADBAND 에서 유도한다 — 둘이 따로 놀면 |PID|=span 에서 ESC 한계를 넘는다.
static const double YAW_FF_SLOPE = 1.0 - YAW_FF_DEADBAND / (ESC_MAX - ESC_NEUTRAL);
// EPS = 이 아래는 0 으로 (불감대 근처 채터링 억제). 권장 대역 3~6 counts 중 4.0 을 골랐다:
//   · 하한 근거 — 계측 바닥에서 멀다. gyro sd 0.0041 rad/s -> P 경로 잡음 0.04 counts.
//     4.0 은 그 100배라 잡음만으로 켜지지 않는다.
//   · 상한 근거 — P 항만 볼 때 무시되는 각도 폭이 EPS/(6.5*10) rad = 3.5° 로 4° 아래다.
//     (적분기가 나머지를 메우므로 정상상태 각도 문턱은 0 이다 — §3-5 replay 표)
//   · 실측 대비 — 같은 830 s 구간의 평균 |PID_yaw| 가 11.4 라 통상 외란 보정은 그대로 산다.
// 첫 수조에서 작은 한계주기가 보이면 6.0 까지 올려라(출력 시간 66%->55%, 실효 출력 8.3->7.9).
static const double YAW_FF_EPS = 4.0;
// 적분기 클램프. 옛 ±100 은 span 300 의 1/3 이라, FF 로 적분기가 처음 출력에 닿는 순간
// 최대 145 counts 의 상시 편향이 된다. ±30 으로 낮춘다(§3-6 2순위).
static const double YAW_I_CLAMP = 30.0;

// ── depth PWM ──
static const int DEPTH_PWM_MIN = 1350;
static const int DEPTH_PWM_MAX = 1650;
static const int DEPTH_BIAS = 30;

// ── gripper Timer1 ──
static const uint16_t GRIPPER_OPEN = 450;
static const uint16_t GRIPPER_STOP = 350;
static const uint16_t GRIPPER_CLOSE = 300;
static const uint16_t GRIPPER_ICR1 = 4999;

// ── yaw unwrap (원본 리터럴 그대로 — M_PI 등으로 바꾸지 말 것) ──
// 타입은 double로 둔다: 원본 리터럴이 double이고 yaw 식이 double로 평가됨.
// float(4.712388f)로 바꾸면 비교/곱셈이 float 정밀도로 떨어져 거동이 미세하게 바뀜.
static const double YAW_WRAP_THRESH = 4.712388;  // 원본 4.712388 (=3*PI/2)
static const double YAW_PI = 3.141592;           // 원본 3.141592

// ── MIP AHRS 프로토콜 ──
static const uint8_t MIP_SYNC1 = 0x75;
static const uint8_t MIP_SYNC2 = 0x65;
static const uint8_t MIP_DESC  = 0x82;
static const int MIP_PACKET_LEN = 48;

// ── baud 주석 (혼동 방지) ──
// AHRS(USART1) + ESC(USART2) = UBRR=8 → 16e6/(16*9) = 111111 baud.
// rosserial PC bridge(USART0, launch) = 76800 baud. 서로 다른 링크 — "불일치" 아님.

#endif // AGENT_CONFIG_H
