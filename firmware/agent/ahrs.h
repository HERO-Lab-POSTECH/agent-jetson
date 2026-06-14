// ahrs.h — MIP(3DM-GX5) AHRS 파싱 모듈 (agent.ino에서 분리, 산술 글자단위 보존)
// 보드 avr-g++ = C++98 (__cplusplus=199711L) → constexpr/auto/range-for 금지, static const만.
//
// ISR(USART1_RX_vect)는 AVR vector 등록 명확성을 위해 메인 agent.ino에 유지한다.
// 본문 파싱 산술(패킷 완성 시 u_data 추출 + yaw unwrap)만 ahrs_parse_packet()로 이동.
// 공유 상태의 *정의*는 agent.ino에 있고, 여기서는 extern 선언만 제공한다.
#ifndef AGENT_AHRS_H
#define AGENT_AHRS_H
#include <stdint.h>
#include <Arduino.h>   // boolean 등
#include "config.h"

// AHRS 공유 상태 (정의는 agent.ino, 여기선 extern 선언)
extern volatile int serial_count;
extern volatile uint8_t inputString[100];
extern volatile boolean stringComplete;
extern volatile uint8_t imu_check;

extern volatile float roll, pitch, yaw;
extern volatile float yaw_temp;
extern volatile float acc_roll, acc_pitch, acc_yaw;
extern volatile float acc_x, acc_y, acc_z;
extern volatile int acc_count;
extern volatile uint8_t ahrs_valid1, ahrs_valid2, ahrs_valid3;

extern volatile uint8_t yaw_calid_command;
extern volatile char yaw_calib;
extern volatile float pre_yaw;
extern volatile float yaw_calib2;

extern volatile int check_hz;

void UART1_write(char ch);
void ahrs_init();          // AHRS MIP init command 시퀀스 (원본 Initialization() 내)
void ahrs_parse_packet();  // 패킷 완성 시 필드 추출 + yaw unwrap (원본 ISR 파싱 블록)
#endif // AGENT_AHRS_H
