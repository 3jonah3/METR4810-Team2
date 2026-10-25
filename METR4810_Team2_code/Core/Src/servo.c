/**
 * @file servo.c
 * TIM4 servo pulse targets. Driver calls use absolute angles or microseconds;
 * robot_control applies calibration before calling this module. Register
 * preloads preserve active pulses while targets change.
 */

#include "servo.h"
#include "main.h"

static TIM_HandleTypeDef* servoTimer = NULL;

static const uint32_t channels[2] = {
    TIM_CHANNEL_2,  // Servo 1: PB7
    TIM_CHANNEL_1   // Servo 2: PB6
};

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
HAL_StatusTypeDef servo(uint8_t servo_ID, int angle)
{
    if (servoTimer == NULL || servo_ID < 1 || servo_ID > 2
            || angle < 0 || angle > SERVO_MAX_ANGLE) {
        return HAL_ERROR;
    }

    return Servo_SetPulseUs(servo_ID, SERVO_ANGLE_US(angle));
}

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
HAL_StatusTypeDef Servo_SetPulseUs(uint8_t servo_ID, unsigned pulse_us)
{
    if (!servoTimer || servo_ID < 1 || servo_ID > 2
            || pulse_us < SERVO_MIN_PULSE_US
            || pulse_us > SERVO_MAX_PULSE_US) {
        return HAL_ERROR;
    }
    // Existing TIM4 configuration: 1 timer count = 1 microsecond.
    __HAL_TIM_SET_COMPARE(servoTimer, channels[servo_ID - 1], pulse_us);

    return HAL_OK;
}

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
void Servo_Init(TIM_HandleTypeDef* timer)
{
    if (timer == NULL || timer->Instance != TIM4) {
        Error_Handler();
        return;
    }

    servoTimer = timer;

    // Start at the centre; PWM stays off until explicitly started.
    if (servo(1, SERVO_CENTRE_ANGLE) != HAL_OK
            || servo(2, SERVO_CENTRE_ANGLE) != HAL_OK) {
        Error_Handler();
        return;
    }

    HAL_TIM_GenerateEvent(servoTimer, TIM_EVENTSOURCE_UPDATE);

    Servo_Stop();
}

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
HAL_StatusTypeDef Servo_Start(uint8_t servo_ID)
{
    if (!servoTimer || servo_ID < 1 || servo_ID > 2) {
        return HAL_ERROR;
    }
    uint32_t channel = channels[servo_ID - 1];
    if (TIM_CHANNEL_STATE_GET(servoTimer, channel)
            == HAL_TIM_CHANNEL_STATE_BUSY) {
        return HAL_OK;
    }
    /* Load the preloaded target before enabling, but only while TIM4 is idle:
     * an update event resets the counter and would stretch a pulse on a servo
     * that is running.
     */
    if (TIM_CHANNEL_STATE_GET(servoTimer, channels[0])
                    != HAL_TIM_CHANNEL_STATE_BUSY
            && TIM_CHANNEL_STATE_GET(servoTimer, channels[1])
                    != HAL_TIM_CHANNEL_STATE_BUSY) {
        HAL_TIM_GenerateEvent(servoTimer, TIM_EVENTSOURCE_UPDATE);
    }
    return HAL_TIM_PWM_Start(servoTimer, channel);
}

/* Servo_Stop()
 * ------------
 * Disable both servo outputs if initialised. Used at startup only: cutting
 * an active pulse short can move the servo.
 *
 * Returns: None.
 * Global variables modified: TIM4 output state.
 * Errors: None.
 */
void Servo_Stop(void)
{
    if (!servoTimer) {
        return;
    }
    HAL_TIM_PWM_Stop(servoTimer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(servoTimer, TIM_CHANNEL_2);
}
