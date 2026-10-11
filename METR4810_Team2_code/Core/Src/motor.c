#include "motor.h"
#include "main.h"  // Error_Handler()

static TIM_HandleTypeDef *motorTimer;

void Motor1_Init(TIM_HandleTypeDef *timer)
{
    motorTimer = timer;

    Motor1_SetDuty(0);
    HAL_TIM_GenerateEvent(motorTimer, TIM_EVENTSOURCE_UPDATE);

    if (HAL_TIM_PWM_Start(motorTimer, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(motorTimer, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
}

void Motor1_SetDuty(int percent)
{
    if (percent > 100)  percent = 100;
    if (percent < -100) percent = -100;

    uint32_t magnitude =
        (uint32_t)(percent < 0 ? -percent : percent);

    uint32_t pulse =
        ((__HAL_TIM_GET_AUTORELOAD(motorTimer) + 1U)
         * magnitude) / 100U;

    if (percent > 0)
    {
        __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, pulse);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, pulse);
    }
}
