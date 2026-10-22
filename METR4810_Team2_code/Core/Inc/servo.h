#ifndef SERVO_H
#define SERVO_H

#include "stm32g4xx_hal.h"
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_PULSE_US 500U
#define SERVO_MAX_PULSE_US 2500U

/* Servo 1 is PB7 (TIM4 CH2) and servo 2 is PB6 (TIM4 CH1). TIM4 ticks once per
   microsecond with a 20 ms period. */

/* Set both servos to 90 degrees with their PWM off. */
void Servo_Init(TIM_HandleTypeDef *timer);
/* Turn on one servo's PWM. Does nothing if it is already on. */
HAL_StatusTypeDef Servo_Start(uint8_t servo_ID);
/* Turn off both PWM outputs. This can cut a pulse short and throw the servo, so the
   firmware only calls it at start-up, before either servo has been started. */
void Servo_Stop(void);
/* Nominal angle 0..SERVO_MAX_ANGLE, mapped linearly onto 500..2500 us. */
HAL_StatusTypeDef servo(uint8_t servo_ID, int angle);
/* Sets pulse width; call Servo_Start(id) to enable PWM. */
HAL_StatusTypeDef Servo_SetPulseUs(uint8_t servo_ID, unsigned pulse_us);

#endif
