/**
 * @file motor.c
 * Signed-duty drill bridge driver on TIM1. Main-loop controller commands
 * choose polarity; zero duty leaves both bridge PWM outputs inactive.
 */

#include "motor.h"
#include "main.h"  // Error_Handler()

static TIM_HandleTypeDef* motorTimer;

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
void Motor1_Init(TIM_HandleTypeDef* timer)
{
    motorTimer = timer;

    Motor1_SetDuty(0);
    HAL_TIM_GenerateEvent(motorTimer, TIM_EVENTSOURCE_UPDATE);

    if (HAL_TIM_PWM_Start(motorTimer, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(motorTimer, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
}

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
void Motor1_SetDuty(int percent)
{
    if (percent > 100) {
        percent = 100;
    }
    if (percent < -100) {
        percent = -100;
    }

    uint32_t magnitude = (uint32_t)(percent < 0 ? -percent : percent);

    uint32_t pulse = ((__HAL_TIM_GET_AUTORELOAD(motorTimer) + 1U)
            * magnitude) / 100U;

    if (percent > 0) {
        __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, pulse);
    } else {
        __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, pulse);
    }
}
