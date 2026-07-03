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
  // NO blocking wait here (user's final decision, field-verified 2026-07-03).
  // ESC arming needs wall-clock time after relay HIGH, but that happens in the
  // BACKGROUND — the ESC boots off its own power rail while loop() keeps running.
  // A delay(5000) here froze the whole loop: rosserial spinOnce, the B2 watchdog,
  // and teleop all stalled for 5s, dropping joint/teleop commands and desyncing
  // the rosserial session. That, not "ESC not arming", was the relay regression.
  // Keep the refresh below so loop() does not spuriously NEUTRAL on relay toggle.
  last_rl_msg_ms = millis();
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
