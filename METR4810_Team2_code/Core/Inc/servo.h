#ifndef SERVO_H
#define SERVO_H

#include "stm32g4xx_hal.h"
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_PULSE_US 500U
#define SERVO_MAX_PULSE_US 2500U
/* Nominal angle (0..SERVO_MAX_ANGLE) to pulse width, rounded: 90 gives 1500 us. */
#define SERVO_ANGLE_US(angle) (SERVO_MIN_PULSE_US + ((unsigned)(angle) * \
    (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) + SERVO_MAX_ANGLE / 2) / SERVO_MAX_ANGLE)
/* Centre of the servos as fitted, where the pod is straight. */
#define SERVO_CENTRE_ANGLE 95

/* Servo 1 is PB7 (TIM4 CH2) and servo 2 is PB6 (TIM4 CH1). TIM4 ticks once per
   microsecond with a 20 ms period. */

/* Set both servos to SERVO_CENTRE_ANGLE with their PWM off. */
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
