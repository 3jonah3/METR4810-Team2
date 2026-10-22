#ifndef MOTOR_H
#define MOTOR_H

#include "stm32g4xx_hal.h"

/* DC motor 1 on TIM1: CH1 drives one direction and CH2 the other. */

/* Start both PWM channels at 0 % duty. */
void Motor1_Init(TIM_HandleTypeDef *timer);
/* Positive is forward, negative reverse, 0 stops. Values beyond +/-100 are clamped. */
void Motor1_SetDuty(int percent);

#endif

