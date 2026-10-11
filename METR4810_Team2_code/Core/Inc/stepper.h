#ifndef STEPPER_H
#define STEPPER_H

#include "stm32g4xx_hal.h"

void Stepper1_Init(TIM_HandleTypeDef *timer);
void Stepper1_Start(uint8_t direction);
void Stepper1_Stop(void);
void Stepper1_SetSpeed(uint32_t speed);
void Stepper1_Move(uint8_t direction, uint32_t steps);
uint8_t Stepper1_IsBusy(void);
uint32_t Stepper1_Remaining(void);

#endif
