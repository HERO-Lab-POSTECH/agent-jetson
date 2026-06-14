// config.h — 매직넘버·핀 선언 (값은 원본 agent.ino 리터럴 그대로 보존)
// 보드 Arduino 툴체인이 C++98로 컴파일하므로 constexpr 대신 static const 사용
// (constexpr은 C++11 전용 → 보드에서 컴파일 불가). 생성 코드는 리터럴과 동일.
#ifndef AGENT_CONFIG_H
#define AGENT_CONFIG_H
#include <stdint.h>

// ── 핀 (원본 #define) ──
static const uint8_t PIN_RELAY = 15;
static const uint8_t PIN_GRIPPER_SIG = 20;
static const uint8_t PIN_LED_SIG = 21;
static const uint8_t PIN_RTS = 10;

// ── ESC PWM ──
static const int ESC_NEUTRAL = 1500;
static const int ESC_MIN = 1200;
static const int ESC_MAX = 1800;

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
