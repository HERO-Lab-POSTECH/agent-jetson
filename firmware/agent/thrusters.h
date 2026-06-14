#ifndef AGENT_THRUSTERS_H
#define AGENT_THRUSTERS_H
#include <stdint.h>
#include <Arduino.h>
// 스러스터 PWM 출력 전역 (정의는 agent.ino, 여기선 extern)
extern volatile int pwm_m0, pid_pwm_m0;
extern volatile int pwm_m1, pid_pwm_m1;
extern volatile int pwm_m2, pid_pwm_m2;
extern volatile int pwm_m3, pid_pwm_m3;
extern volatile int pwm_m4, pid_pwm_m4;
extern volatile int pwm_m5, pid_pwm_m5;
void UART2_write(char ch);
void esc_input(uint8_t ID, uint16_t pwm0, uint16_t pwm1, uint16_t pwm2);
#endif
