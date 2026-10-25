/**
 * @file stepper.c
 * TIM16 STEP/DIR driver. Main code starts/stops moves; the update ISR counts
 * pulses and rearms one-pulse mode. Shared counters are volatile; stop masks
 * interrupts while accounting for already-emitted edges. No limit logic here.
 */

#include "stepper.h"
#include "main.h"

static TIM_HandleTypeDef* stepTimer = NULL;
static volatile uint8_t running = 0;
static volatile uint32_t remaining;
static volatile uint32_t total_pulses;

/* Stepper1_Init()
 * ---------------
 * Bind the STEP/DIR driver to TIM16, stop PWM and clear its counters.
 *
 * timer: Non-NULL initialised TIM16 handle.
 *
 * Returns: None.
 * Global variables modified: stepTimer, running, remaining, total_pulses and
 * timer/DIR registers.
 * Errors: Invalid handle or PWM-stop failure enters Error_Handler().
 */
void Stepper1_Init(TIM_HandleTypeDef* timer)
{
    if (timer == NULL || timer->Instance != TIM16) {
        Error_Handler();
        return;
    }

    stepTimer = timer;
    running = 0;
    remaining = 0;
    total_pulses = 0;

    // Keep the pulse output stopped until Start() is called.
    if (HAL_TIM_PWM_Stop(stepTimer, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

/* Stepper1_Start()
 * ----------------
 * Start continuous pulses from main context. Wait 1 ms for direction setup;
 * ignore the request if already running. Does not read limits.
 *
 * direction: Zero drives toward SW4; nonzero drives toward SW1.
 *
 * Returns: None.
 * Global variables modified: running, remaining, TIM16 registers and DIR
 * GPIO.
 * Errors: Missing initialisation or PWM-start failure enters
 * Error_Handler().
 */
void Stepper1_Start(uint8_t direction)
{
    if (stepTimer == NULL) {
        Error_Handler();
        return;
    }

    // Stop first before changing direction.
    // Call Stop() and allow the motor to settle before reversing.
    if (running) {
        return;
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,
            direction ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Direction setup delay. Call from main code, not an interrupt.
    HAL_Delay(1);

    __HAL_TIM_SET_COUNTER(stepTimer, 0);

    /* PWM2 starts LOW. Each overflow completes one hardware-generated pulse. */
    CLEAR_BIT(stepTimer->Instance->CR1, TIM_CR1_OPM);
    MODIFY_REG(stepTimer->Instance->CCMR1, TIM_CCMR1_OC1M, TIM_OCMODE_PWM2);
    remaining = 0;
    __HAL_TIM_CLEAR_FLAG(stepTimer, TIM_FLAG_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
    running = 1;
    __HAL_TIM_ENABLE_IT(stepTimer, TIM_IT_UPDATE);

    if (HAL_TIM_PWM_Start(stepTimer, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
        return;
    }

    running = 1;
}

/* Stepper1_StopRemaining()
 * ------------------------
 * Stop atomically and account for already-emitted edges. Restore the
 * incoming interrupt mask; safe in main or the timer callback.
 *
 * Returns: Unissued counted pulses; zero for continuous, idle or
 * uninitialised.
 * Global variables modified: running, remaining, total_pulses and TIM16
 * registers.
 * Errors: PWM-stop failure enters Error_Handler().
 */
uint32_t Stepper1_StopRemaining(void)
{
    uint32_t left = 0;
    if (stepTimer == NULL) {
        return 0;
    }

    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    CLEAR_BIT(stepTimer->Instance->CR1, TIM_CR1_CEN);
    __HAL_TIM_DISABLE_IT(stepTimer, TIM_IT_UPDATE);
    if (running) {
        /* Count a pending overflow and final rising edge before stopping.
           Each pulse counted here is also one fewer left in a counted move. */
        left = remaining;
        if (__HAL_TIM_GET_FLAG(stepTimer, TIM_FLAG_UPDATE)) {
            ++total_pulses;
            if (left) {
                --left;
            }
        }
        if (__HAL_TIM_GET_COUNTER(stepTimer)
                >= __HAL_TIM_GET_COMPARE(stepTimer, TIM_CHANNEL_1)) {
            ++total_pulses;
            if (left) {
                --left;
            }
        }
    }
    __HAL_TIM_CLEAR_FLAG(stepTimer, TIM_FLAG_UPDATE);
    running = 0;
    remaining = 0;
    __set_PRIMASK(mask);

    if (HAL_TIM_PWM_Stop(stepTimer, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
        return 0;
    }

    running = 0;
    return left;
}

/* Stepper1_Stop()
 * ---------------
 * Stop and discard pending movement, retaining the cumulative count. Account
 * for already-emitted edges through Stepper1_StopRemaining().
 *
 * Returns: None.
 * Global variables modified: running, remaining, total_pulses and TIM16
 * registers.
 * Errors: PWM-stop failure enters Error_Handler().
 */
void Stepper1_Stop(void)
{
    (void)Stepper1_StopRemaining();
}

/* Stepper1_SetSpeed()
 * -------------------
 * Set the stopped timer period and 50-percent pulse duty. Does not start
 * movement.
 *
 * speed: Requested STEP pulses/s, 16..2000, using a 1 MHz timer clock.
 *
 * Returns: None.
 * Global variables modified: TIM16 period/compare registers.
 * Errors: Invalid speed, uninitialised or running driver: request is
 * ignored.
 */
void Stepper1_SetSpeed(uint32_t speed)
{
    if (!stepTimer || running || speed < 16U || speed > 2000U) {
        return;
    }
    uint32_t period = 1000000U / speed;
    __HAL_TIM_SET_AUTORELOAD(stepTimer, period - 1U);
    __HAL_TIM_SET_COMPARE(stepTimer, TIM_CHANNEL_1, period / 2U);
    HAL_TIM_GenerateEvent(stepTimer, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(stepTimer, TIM_FLAG_UPDATE);
}

/* Stepper1_Move()
 * ---------------
 * Start a hardware-counted move from main context, after a 1 ms direction
 * delay. The controller handles limits separately.
 *
 * direction: Zero drives toward SW4; nonzero drives toward SW1.
 * steps: Number of STEP pulses to emit.
 *
 * Returns: None.
 * Global variables modified: running, remaining, TIM16 registers and DIR
 * GPIO.
 * Errors: Zero count, uninitialised or busy driver is ignored; PWM failure
 *     enters Error_Handler().
 */
void Stepper1_Move(uint8_t direction, uint32_t steps)
{
    if (!stepTimer || !steps || running) {
        return;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,
            direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
    if (HAL_TIM_PWM_Start(stepTimer, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}

/* HAL_TIM_PeriodElapsedCallback()
 * -------------------------------
 * Count a STEP pulse and rearm one-pulse mode only while pulses remain.
 * Ignore unrelated timers and stopped movement.
 *
 * timer: HAL handle whose timer overflowed.
 *
 * Returns: None.
 * Global variables modified: total_pulses, remaining, running and TIM16
 * registers.
 * Errors: A final counted pulse invokes Stepper1_Stop(); HAL failure halts.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* timer)
{
    if (timer != stepTimer || !running) {
        return;
    }
    ++total_pulses;
    if (remaining) {
        if (--remaining == 0U) {
            Stepper1_Stop();
        } else {
            SET_BIT(timer->Instance->CR1, TIM_CR1_CEN);
        }
    }
}

/* Stepper1_IsBusy()
 * -----------------
 * Read the interrupt-owned movement flag.
 *
 * Returns: Nonzero while emitting continuous or counted pulses; zero when
 * idle.
 * Global variables modified: None.
 * Errors: None.
 */
uint8_t Stepper1_IsBusy(void)
{
    return running;
}

/* Stepper1_Remaining()
 * --------------------
 * Read the interrupt-owned remaining pulse count.
 *
 * Returns: Unissued counted pulses; zero in continuous or stopped mode.
 * Global variables modified: None.
 * Errors: None.
 */
uint32_t Stepper1_Remaining(void)
{
    return remaining;
}

/* Stepper1_TotalPulses()
 * ----------------------
 * Read commanded travel, not encoder feedback.
 *
 * Returns: Emitted STEP pulses since initialisation, wrapping at UINT32_MAX.
 * Global variables modified: None.
 * Errors: None.
 */
uint32_t Stepper1_TotalPulses(void)
{
    return total_pulses;
}
