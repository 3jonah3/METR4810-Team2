#include "servo.h"
#include "main.h"

static TIM_HandleTypeDef *servoTimer = NULL;

/* Nominal 0-180 degree mapping for the 500-2500 us servos. */

static const uint32_t channels[2] = {
    TIM_CHANNEL_2,  // Servo 1: PB7
    TIM_CHANNEL_1   // Servo 2: PB6
};

HAL_StatusTypeDef servo(uint8_t servo_ID, int angle)
{
    if (servoTimer == NULL ||
        servo_ID < 1 || servo_ID > 2 ||
        angle < 0 || angle > SERVO_MAX_ANGLE)
    {
        return HAL_ERROR;
    }

    uint32_t pulseUs = SERVO_MIN_PULSE_US
        + ((uint32_t)angle *
           (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)
           + SERVO_MAX_ANGLE / 2) / SERVO_MAX_ANGLE;

    return Servo_SetPulseUs(servo_ID, pulseUs);
}

HAL_StatusTypeDef Servo_SetPulseUs(uint8_t servo_ID, unsigned pulse_us)
{
    if (!servoTimer || servo_ID < 1 || servo_ID > 2 ||
        pulse_us < SERVO_MIN_PULSE_US || pulse_us > SERVO_MAX_PULSE_US)
        return HAL_ERROR;
    // Existing TIM4 configuration: 1 timer count = 1 microsecond.
    __HAL_TIM_SET_COMPARE(servoTimer, channels[servo_ID - 1], pulse_us);

    return HAL_OK;
}

void Servo_Init(TIM_HandleTypeDef *timer)
{
    if (timer == NULL || timer->Instance != TIM4)
    {
        Error_Handler();
        return;
    }

    servoTimer = timer;

    // Centre is 90 degrees (1500 us); PWM stays off until explicitly started.
    if (servo(1, 90) != HAL_OK || servo(2, 90) != HAL_OK)
    {
        Error_Handler();
        return;
    }

    HAL_TIM_GenerateEvent(servoTimer, TIM_EVENTSOURCE_UPDATE);

    Servo_Stop();
}

HAL_StatusTypeDef Servo_Start(uint8_t servo_ID)
{
    if (!servoTimer || servo_ID < 1 || servo_ID > 2) return HAL_ERROR;
    uint32_t channel = channels[servo_ID - 1];
    if (TIM_CHANNEL_STATE_GET(servoTimer, channel) == HAL_TIM_CHANNEL_STATE_BUSY)
        return HAL_OK;
    /* Load the preloaded target before enabling, but only while TIM4 is idle: an update
       event resets the counter and would stretch a pulse on a servo that is running. */
    if (TIM_CHANNEL_STATE_GET(servoTimer, channels[0]) != HAL_TIM_CHANNEL_STATE_BUSY &&
        TIM_CHANNEL_STATE_GET(servoTimer, channels[1]) != HAL_TIM_CHANNEL_STATE_BUSY)
        HAL_TIM_GenerateEvent(servoTimer, TIM_EVENTSOURCE_UPDATE);
    return HAL_TIM_PWM_Start(servoTimer, channel);
}

void Servo_Stop(void)
{
    if (!servoTimer) return;
    HAL_TIM_PWM_Stop(servoTimer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(servoTimer, TIM_CHANNEL_2);
}
