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
  // 원본의 delay(5000) 제거: relay-ON 후 후속 동작이 없는 단순 5초 블로킹이었음.
  // 이 5초 동안 nh.spinOnce()/PID 루프 전체가 freeze되어 거동을 해쳤으므로 비블로킹화(=제거).
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
