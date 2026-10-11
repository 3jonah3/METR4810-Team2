#ifndef SERVO_H
#define SERVO_H

#include "stm32g4xx_hal.h"
#define SERVO_MAX_ANGLE 120

void Servo_Init(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef Servo_Start(uint8_t servo_ID);
void Servo_Stop(void);
HAL_StatusTypeDef servo(uint8_t servo_ID, int angle);

#endif
