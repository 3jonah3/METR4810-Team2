#include "servo.h"
#include "main.h"

static TIM_HandleTypeDef *servoTimer = NULL;

// Nominal range — check against your servo's specifications.
#define SERVO_MIN_US  1000U
#define SERVO_MAX_US  2000U

void Servo_Init(TIM_HandleTypeDef *timer)
{
    if (timer == NULL || timer->Instance != TIM4)
    {
        Error_Handler();
        return;
    }

    servoTimer = timer;

    // Start both servos at the nominal centre position.
    __HAL_TIM_SET_COMPARE(servoTimer, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(servoTimer, TIM_CHANNEL_2, 1500);

    HAL_TIM_GenerateEvent(servoTimer, TIM_EVENTSOURCE_UPDATE);

    if (HAL_TIM_PWM_Start(servoTimer, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(servoTimer, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
}

HAL_StatusTypeDef servo(uint8_t servo_ID, int angle)
{
    if (servoTimer == NULL || angle < 0 || angle > 180)
    {
        return HAL_ERROR;
    }

    uint32_t channel;

    switch (servo_ID)
    {
        case 1:
            channel = TIM_CHANNEL_2;  // PB7: Servo 1
            break;

        case 2:
            channel = TIM_CHANNEL_1;  // PB6: Servo 2
            break;

        default:
            return HAL_ERROR;
    }

    // TIM4 counter must run at 1 MHz: one count = 1 us.
    uint32_t pulse = SERVO_MIN_US
        + ((uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180U;

    __HAL_TIM_SET_COMPARE(servoTimer, channel, pulse);

    return HAL_OK;
}
