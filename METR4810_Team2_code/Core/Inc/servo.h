#ifndef SERVO_H
#define SERVO_H

#include "stm32g4xx_hal.h"
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_PULSE_US 500U
#define SERVO_MAX_PULSE_US 2500U

void Servo_Init(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef Servo_Start(uint8_t servo_ID);
void Servo_Stop(void);
HAL_StatusTypeDef servo(uint8_t servo_ID, int angle);
/* Sets pulse width; call Servo_Start(id) to enable PWM. */
HAL_StatusTypeDef Servo_SetPulseUs(uint8_t servo_ID, unsigned pulse_us);

#endif
