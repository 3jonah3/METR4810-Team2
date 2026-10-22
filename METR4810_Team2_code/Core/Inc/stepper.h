#ifndef STEPPER_H
#define STEPPER_H

#include "stm32g4xx_hal.h"

/* Stepper 1: STEP pulses from TIM16 CH1, DIR on PA4 (direction 0 drives toward SW4).
   Each timer overflow is one complete STEP pulse. */

void Stepper1_Init(TIM_HandleTypeDef *timer);
/* Run continuously until Stepper1_Stop. Ignored while already running. Main code only. */
void Stepper1_Start(uint8_t direction);
/* Stop at once, counting a pulse that has already started. The timer interrupt also
   calls this when a counted move ends. */
void Stepper1_Stop(void);
/* Stepper1_Stop that also returns the counted-move pulses still to do (0 in continuous
   mode), with any pulse it has just counted already taken off. */
uint32_t Stepper1_StopRemaining(void);
/* 16..2000 pulses per second. Ignored while running. */
void Stepper1_SetSpeed(uint32_t speed);
/* Emit exactly `steps` pulses, then stop by itself. Ignored while running. Main code only. */
void Stepper1_Move(uint8_t direction, uint32_t steps);
uint8_t Stepper1_IsBusy(void);
/* Pulses left in a counted move; 0 when stopped or running continuously. */
uint32_t Stepper1_Remaining(void);
/* Cumulative emitted STEP pulses, including continuous motion; wraps at UINT32_MAX. */
uint32_t Stepper1_TotalPulses(void);

#endif
