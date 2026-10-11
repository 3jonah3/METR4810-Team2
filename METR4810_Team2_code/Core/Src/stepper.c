#include "stepper.h"
#include "main.h"

static TIM_HandleTypeDef *stepTimer = NULL;
static uint8_t running = 0;

void Stepper1_Init(TIM_HandleTypeDef *timer)
{
    if (timer == NULL || timer->Instance != TIM16)
    {
        Error_Handler();
        return;
    }

    stepTimer = timer;
    running = 0;

    // Keep the pulse output stopped until Start() is called.
    if (HAL_TIM_PWM_Stop(stepTimer, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void Stepper1_Start(uint8_t direction)
{
    if (stepTimer == NULL)
    {
        Error_Handler();
        return;
    }

    // Stop first before changing direction.
    // Call Stop() and allow the motor to settle before reversing.
    if (running)
    {
        return;
    }

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_4,
        direction ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    // Direction setup delay. Call from main code, not an interrupt.
    HAL_Delay(1);

    __HAL_TIM_SET_COUNTER(stepTimer, 0);

    if (HAL_TIM_PWM_Start(stepTimer, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
        return;
    }

    running = 1;
}

void Stepper1_Stop(void)
{
    if (stepTimer == NULL)
    {
        return;
    }

    if (HAL_TIM_PWM_Stop(stepTimer, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
        return;
    }

    running = 0;
}
