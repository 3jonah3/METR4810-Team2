#ifndef SERVO_H
#define SERVO_H

#include "stm32g4xx_hal.h"
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_PULSE_US 500U
#define SERVO_MAX_PULSE_US 2500U
/* Convert nominal degrees to rounded microseconds; 90 gives 1500 us. */
#define SERVO_ANGLE_US(angle) \
    (SERVO_MIN_PULSE_US + ((unsigned)(angle) \
            * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) \
            + SERVO_MAX_ANGLE / 2) / SERVO_MAX_ANGLE)

/* Centre of the servos as fitted, where the pod is straight. */
#define SERVO_CENTRE_ANGLE 95

/* Servo 1 is PB7 (TIM4 CH2) and servo 2 is PB6 (TIM4 CH1). TIM4 ticks once per
   microsecond with a 20 ms period. */

/* Servo_Init()
 * ------------
 * Bind TIM4, preload the legacy centre targets, then disable both PWM
 * outputs.
 *
 * timer: Initialised TIM4 handle, 1 MHz clock and 20 ms period.
 *
 * Returns: None.
 * Global variables modified: servoTimer and TIM4 compare/output state.
 * Errors: Invalid handle or failed target setup enters Error_Handler().
 */
void Servo_Init(TIM_HandleTypeDef* timer);

/* Servo_Start()
 * -------------
 * Enable one servo without disturbing any active PWM pulse.
 *
 * servo_ID: 1 for PB7/CH2 or 2 for PB6/CH1.
 *
 * Returns: HAL_OK if already active/started, otherwise a HAL error.
 * Global variables modified: TIM4 output state; targets are unchanged.
 * Errors: Invalid id, missing initialisation or HAL start failure.
 */
HAL_StatusTypeDef Servo_Start(uint8_t servo_ID);

/* Servo_Stop()
 * ------------
 * Disable both servo outputs if initialised. Used at startup only: cutting
 * an active pulse short can move the servo.
 *
 * Returns: None.
 * Global variables modified: TIM4 output state.
 * Errors: None.
 */
void Servo_Stop(void);

/* servo()
 * -------
 * Convert an absolute nominal angle to a PWM target without enabling output.
 *
 * servo_ID: Servo 1 or 2.
 * angle: 0..180 degrees, mapped to 500..2500 microseconds.
 *
 * Returns: HAL_OK on success; HAL_ERROR otherwise.
 * Global variables modified: TIM4 compare preload.
 * Errors: Invalid id/angle or missing initialisation.
 */
HAL_StatusTypeDef servo(uint8_t servo_ID, int angle);

/* Servo_SetPulseUs()
 * ------------------
 * Set the absolute PWM target without enabling output. The preload preserves
 * any pulse already in progress.
 *
 * servo_ID: Servo 1 or 2.
 * pulse_us: 500..2500 microseconds.
 *
 * Returns: HAL_OK on success; HAL_ERROR otherwise.
 * Global variables modified: TIM4 compare preload.
 * Errors: Invalid id/pulse width or missing initialisation.
 */
HAL_StatusTypeDef Servo_SetPulseUs(uint8_t servo_ID, unsigned pulse_us);

#endif
