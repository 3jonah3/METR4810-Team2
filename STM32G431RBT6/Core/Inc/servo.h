#ifndef SERVO_H
#define SERVO_H

#include "stm32g4xx_hal.h"

void Servo_Init(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef servo(uint8_t servo_ID, int angle);

#endif
