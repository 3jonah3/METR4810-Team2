#include "servo.h"
#include "main.h"

static TIM_HandleTypeDef *servoTimer = NULL;

/*
 * Starting pulse limits, NOT specified by the supplied datasheet.
 * Calibrate each servo separately if the measured travel differs.
 */
static const uint16_t minPulseUs[2] = {1000, 1000};
static const uint16_t maxPulseUs[2] = {2000, 2000};

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

    uint8_t index = servo_ID - 1;

    uint32_t pulseUs = minPulseUs[index]
        + ((uint32_t)angle *
           (maxPulseUs[index] - minPulseUs[index])
           + SERVO_MAX_ANGLE / 2) / SERVO_MAX_ANGLE;

    // Existing TIM4 configuration: 1 timer count = 1 microsecond.
    __HAL_TIM_SET_COMPARE(servoTimer, channels[index], pulseUs);

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

    // Centre is now 60° within a nominal 0–120° range.
    if (servo(1, 60) != HAL_OK || servo(2, 60) != HAL_OK)
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
    /* Apply preloaded target before enabling an inactive channel. */
    HAL_TIM_GenerateEvent(servoTimer, TIM_EVENTSOURCE_UPDATE);
    return HAL_TIM_PWM_Start(servoTimer, channel);
}

void Servo_Stop(void)
{
    if (!servoTimer) return;
    HAL_TIM_PWM_Stop(servoTimer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(servoTimer, TIM_CHANNEL_2);
}
