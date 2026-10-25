#ifndef STEPPER_H
#define STEPPER_H

#include "stm32g4xx_hal.h"

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
void Stepper1_Init(TIM_HandleTypeDef* timer);

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
void Stepper1_Start(uint8_t direction);

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
void Stepper1_Stop(void);

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
uint32_t Stepper1_StopRemaining(void);

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
void Stepper1_SetSpeed(uint32_t speed);

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
void Stepper1_Move(uint8_t direction, uint32_t steps);

/* Stepper1_IsBusy()
 * -----------------
 * Read the interrupt-owned movement flag.
 *
 * Returns: Nonzero while emitting continuous or counted pulses; zero when
 * idle.
 * Global variables modified: None.
 * Errors: None.
 */
uint8_t Stepper1_IsBusy(void);

/* Stepper1_Remaining()
 * --------------------
 * Read the interrupt-owned remaining pulse count.
 *
 * Returns: Unissued counted pulses; zero in continuous or stopped mode.
 * Global variables modified: None.
 * Errors: None.
 */
uint32_t Stepper1_Remaining(void);

/* Stepper1_TotalPulses()
 * ----------------------
 * Read commanded travel, not encoder feedback.
 *
 * Returns: Emitted STEP pulses since initialisation, wrapping at UINT32_MAX.
 * Global variables modified: None.
 * Errors: None.
 */
uint32_t Stepper1_TotalPulses(void);

#endif
