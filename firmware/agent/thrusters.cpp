#include "thrusters.h"

void UART2_write(char ch)
{

  while (!(UCSR2A & 0x20))
    ;

  UDR2 = ch;
}

void esc_input(uint8_t ID, uint16_t pwm0, uint16_t pwm1, uint16_t pwm2)
{
  UART2_write(0xff);
  UART2_write(0xff);
  UART2_write(ID);
  UART2_write(pwm0 / 256);
  UART2_write(pwm0 % 256);
  UART2_write(pwm1 / 256);
  UART2_write(pwm1 % 256);
  UART2_write(pwm2 / 256);
  UART2_write(pwm2 % 256);
  UART2_write(0xfe);
}
