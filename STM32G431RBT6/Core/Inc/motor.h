#ifndef MOTOR_H
#define MOTOR_H

#include "stm32g4xx_hal.h"

void Motor1_Init(TIM_HandleTypeDef *timer);
void Motor1_SetDuty(int percent);

#endif

