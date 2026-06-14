#include <Arduino.h>
#include "io.h"

void io_init()
{
  // basic io init
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_LED_SIG, OUTPUT);
  pinMode(PIN_RTS, OUTPUT);

  // RTS on (rs485 send)
  digitalWrite(PIN_RTS, HIGH);
}

void gripper_init()
{
  // init gripper by servo
  // GRIPPER_SERVO.attach(GRIPPER_SIG);
  DDRB |= 0x20;
  TCCR1A = 0x82;
  TCCR1B = 0x1B;
  TCCR1C = 0;
  ICR1 = GRIPPER_ICR1;

  OCR1A = GRIPPER_STOP;
}

void relay_on()
{
  state_Relay = 1;
  digitalWrite(PIN_RELAY, HIGH);
  // ESC arming 대기 (필수): relay HIGH로 스러스터 ESC에 전원이 인가되면 ESC가
  // 부팅·초기화("띠리리띠띠" 기동음)에 시간이 필요하다. 이 5초간 loop를 멈춰
  // ESC가 끼어듦 없이 arming을 완료하게 한다. 구조화 때 "단순 블로킹"으로 오판해
  // 제거했다가 실기에서 ESC 미기동(monitor relay ON·기동음 없음)으로 발견·복원함.
  delay(5000);
}

void relay_off()
{
  state_Relay = 0;
  digitalWrite(PIN_RELAY, LOW);
}

void laser_on()
{
  state_Laser = 1;
  digitalWrite(PIN_LED_SIG, HIGH);
}

void laser_off()
{
  state_Laser = 0;
  digitalWrite(PIN_LED_SIG, LOW);
}

void gripper_set(uint16_t ocr)
{
  OCR1A = ocr;
}
