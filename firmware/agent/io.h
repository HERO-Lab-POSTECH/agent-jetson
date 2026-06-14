#ifndef AGENT_IO_H
#define AGENT_IO_H
#include "config.h"
extern volatile uint8_t state_Relay, state_Laser;
void io_init();        // pinMode(RELAY/LED/RTS) + RTS HIGH
void gripper_init();   // Timer1 PWM 설정 (DDRB/TCCR1/ICR1/OCR1A)
void relay_on();
void relay_off();
void laser_on();
void laser_off();
void gripper_set(uint16_t ocr);  // OCR1A = ocr
#endif
