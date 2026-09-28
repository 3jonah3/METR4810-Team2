#ifndef STEPPER_H
#define STEPPER_H

#include "stm32g4xx_hal.h"

void Stepper1_Init(TIM_HandleTypeDef *timer);
void Stepper1_Start(uint8_t direction);
void Stepper1_Stop(void);

#endif
