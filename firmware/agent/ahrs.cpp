// ahrs.cpp — MIP(3DM-GX5) AHRS 파싱 구현 (agent.ino에서 cut-paste, 산술 글자단위 보존)
// 상수치환(Task2: YAW_WRAP_THRESH/YAW_PI/MIP_*)만 적용, byte offset·연산은 원본 그대로.
#include "ahrs.h"

// u_data union — 원본 agent.ino 파일스코프 정의 그대로 이동.
// 패킷 완성 파싱(ahrs_parse_packet)에서만 사용하므로 이 TU 파일스코프에 둔다.
// ISR(USART1_RX_vect, 메인 agent.ino 유지)은 u_data를 쓰지 않는다.
volatile union
{
  float real;
  uint32_t base;
} u_data;

void UART1_write(char ch)
{

  while (!(UCSR1A & 0x20))
    ;

  UDR1 = ch;
}

// 원본 ISR(USART1_RX_vect)의 "패킷 완성(serial_count==MIP_PACKET_LEN && inputString[35]==1)"
// 블록 본문을 글자 그대로 이동. ISR의 if 조건/sync 상태머신/reset(else if)은 메인 유지.
void ahrs_parse_packet()
{
  stringComplete = true;
  serial_count = 0;
  imu_check = 0;
  check_hz++;
  u_data.base = 0;
  u_data.base |= ((uint32_t)inputString[9]) << (8 * 0);
  u_data.base |= ((uint32_t)inputString[8]) << (8 * 1);
  u_data.base |= ((uint32_t)inputString[7]) << (8 * 2);
  u_data.base |= ((uint32_t)inputString[6]) << (8 * 3);
  roll = u_data.real;

  u_data.base = 0;
  u_data.base |= ((uint32_t)inputString[13]) << (8 * 0);
  u_data.base |= ((uint32_t)inputString[12]) << (8 * 1);
  u_data.base |= ((uint32_t)inputString[11]) << (8 * 2);
  u_data.base |= ((uint32_t)inputString[10]) << (8 * 3);
  pitch = u_data.real;

  u_data.base = 0;
  u_data.base |= ((uint32_t)inputString[17]) << (8 * 0);
  u_data.base |= ((uint32_t)inputString[16]) << (8 * 1);
  u_data.base |= ((uint32_t)inputString[15]) << (8 * 2);
  u_data.base |= ((uint32_t)inputString[14]) << (8 * 3);
  yaw = u_data.real;
  yaw_temp = u_data.real;
  u_data.base = 0;
  u_data.base |= ((uint32_t)inputString[25]) << (8 * 0);
  u_data.base |= ((uint32_t)inputString[24]) << (8 * 1);
  u_data.base |= ((uint32_t)inputString[23]) << (8 * 2);
  u_data.base |= ((uint32_t)inputString[22]) << (8 * 3);
  acc_roll = u_data.real;

  u_data.base = 0;
  u_data.base |= ((uint32_t)inputString[29]) << (8 * 0);
  u_data.base |= ((uint32_t)inputString[28]) << (8 * 1);
  u_data.base |= ((uint32_t)inputString[27]) << (8 * 2);
  u_data.base |= ((uint32_t)inputString[26]) << (8 * 3);
  acc_pitch = u_data.real;

  u_data.base = 0;
  u_data.base |= ((uint32_t)inputString[33]) << (8 * 0);
  u_data.base |= ((uint32_t)inputString[32]) << (8 * 1);
  u_data.base |= ((uint32_t)inputString[31]) << (8 * 2);
  u_data.base |= ((uint32_t)inputString[30]) << (8 * 3);
  acc_yaw = u_data.real;

  u_data.base = 0;
  u_data.base |= ((uint32_t)inputString[41]) << (8 * 0);
  u_data.base |= ((uint32_t)inputString[40]) << (8 * 1);
  u_data.base |= ((uint32_t)inputString[39]) << (8 * 2);
  u_data.base |= ((uint32_t)inputString[38]) << (8 * 3);
  // if (u_data.real > -0.01 && u_data.real < 0.01)
  acc_x += u_data.real;

  u_data.base = 0;
  u_data.base |= ((uint32_t)inputString[45]) << (8 * 0);
  u_data.base |= ((uint32_t)inputString[44]) << (8 * 1);
  u_data.base |= ((uint32_t)inputString[43]) << (8 * 2);
  u_data.base |= ((uint32_t)inputString[42]) << (8 * 3);
  if (u_data.real > -0.01 && u_data.real < 0.01)
    acc_y += u_data.real;

  u_data.base = 0;
  u_data.base |= ((uint32_t)inputString[49]) << (8 * 0);
  u_data.base |= ((uint32_t)inputString[48]) << (8 * 1);
  u_data.base |= ((uint32_t)inputString[47]) << (8 * 2);
  u_data.base |= ((uint32_t)inputString[46]) << (8 * 3);
  if (u_data.real > -0.01 && u_data.real < 0.01)
    acc_z += u_data.real;
  acc_count++;

  ahrs_valid1 = inputString[19];
  ahrs_valid2 = inputString[35];
  ahrs_valid3 = inputString[51];

  yaw -= yaw_calib2;

  if (yaw_calid_command == 1)
  {
    yaw_calid_command = 0;
    yaw_calib2 = yaw_temp;
    pre_yaw = 0;
    yaw = 0;
    yaw_calib = 0;
  }

  if (yaw - pre_yaw > YAW_WRAP_THRESH)
    yaw_calib--;
  else if (yaw - pre_yaw < -YAW_WRAP_THRESH)
    yaw_calib++;
  pre_yaw = yaw;
  yaw += YAW_PI * yaw_calib * 2;
}

// 원본 Initialization() 내 AHRS MIP init 시퀀스 그대로 이동 (delay 포함).
void ahrs_init()
{
  delay(100);
  UART1_write(0x75);
  UART1_write(0x65);
  UART1_write(0x0C);
  UART1_write(0x05);
  UART1_write(0x05);
  UART1_write(0x11);
  UART1_write(0x01);
  UART1_write(0x03);
  UART1_write(0x01);
  UART1_write(0x06);
  UART1_write(0x1E);
  delay(1000);
}
