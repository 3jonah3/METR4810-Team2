#include "stepper.h"
#include "main.h"

static TIM_HandleTypeDef *stepTimer = NULL;
static volatile uint8_t running = 0;
static volatile uint32_t remaining;

void Stepper1_Init(TIM_HandleTypeDef *timer)
{
    if (timer == NULL || timer->Instance != TIM16)
    {
        Error_Handler();
        return;
    }

    stepTimer = timer;
    running = 0;
    remaining = 0;

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

    /* Continuous travel uses the original PWM1 waveform. */
    CLEAR_BIT(stepTimer->Instance->CR1, TIM_CR1_OPM);
    MODIFY_REG(stepTimer->Instance->CCMR1, TIM_CCMR1_OC1M, TIM_OCMODE_PWM1);

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

    __HAL_TIM_DISABLE_IT(stepTimer, TIM_IT_UPDATE);
    remaining = 0;

    if (HAL_TIM_PWM_Stop(stepTimer, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
        return;
    }

    running = 0;
}

void Stepper1_SetSpeed(uint32_t speed)
{
    if (!stepTimer || running || speed < 16U || speed > 2000U) return;
    uint32_t period = 1000000U / speed;
    __HAL_TIM_SET_AUTORELOAD(stepTimer, period - 1U);
    __HAL_TIM_SET_COMPARE(stepTimer, TIM_CHANNEL_1, period / 2U);
    HAL_TIM_GenerateEvent(stepTimer, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(stepTimer, TIM_FLAG_UPDATE);
}

void Stepper1_Move(uint8_t direction, uint32_t steps)
{
    if (!stepTimer || !steps || running) return;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_Delay(1); /* Existing driver direction setup time; main context only. */
    /* PWM2 starts LOW, rises at CCR1, falls at ARR overflow. OPM then
       stops the counter in hardware, so ISR latency cannot add a pulse.
       Each update ISR rearms exactly one more pulse until count is zero. */
    MODIFY_REG(stepTimer->Instance->CCMR1, TIM_CCMR1_OC1M, TIM_OCMODE_PWM2);
    SET_BIT(stepTimer->Instance->CR1, TIM_CR1_OPM);
    __HAL_TIM_SET_COUNTER(stepTimer, 0);
    __HAL_TIM_CLEAR_FLAG(stepTimer, TIM_FLAG_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
    remaining = steps;
    running = 1;
    __HAL_TIM_ENABLE_IT(stepTimer, TIM_IT_UPDATE);
    if (HAL_TIM_PWM_Start(stepTimer, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer != stepTimer || !remaining) return;
    if (--remaining == 0U) Stepper1_Stop();
    else SET_BIT(timer->Instance->CR1, TIM_CR1_CEN);
}

uint8_t Stepper1_IsBusy(void) { return running; }
uint32_t Stepper1_Remaining(void) { return remaining; }
