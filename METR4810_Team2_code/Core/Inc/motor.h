#ifndef MOTOR_H
#define MOTOR_H

#include "stm32g4xx_hal.h"

/* DC motor 1 on TIM1: CH1 drives one direction and CH2 the other. */

/* Motor1_Init()
 * -------------
 * Bind TIM1 and enable both bridge channels at zero duty. Call once after
 * timer setup.
 *
 * timer: Non-NULL initialised TIM1 handle.
 *
 * Returns: None.
 * Global variables modified: motorTimer and TIM1 registers.
 * Errors: PWM-start failure enters Error_Handler().
 */
void Motor1_Init(TIM_HandleTypeDef* timer);

/* Motor1_SetDuty()
 * ----------------
 * Set signed bridge duty after initialisation. Clear the inactive direction
 * before setting the active channel.
 *
 * percent: Clamped to -100..100; positive uses CH2, negative CH1,
 *     zero clears both.
 *
 * Returns: None.
 * Global variables modified: TIM1 compare registers.
 * Errors: Requires Motor1_Init() to have supplied a valid timer.
 */
void Motor1_SetDuty(int percent);

#endif

