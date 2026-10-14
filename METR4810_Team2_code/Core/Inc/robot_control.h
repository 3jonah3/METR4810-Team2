#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include "stm32g4xx_hal.h"

typedef enum { ROBOT_IDLE, ROBOT_RUNNING, ROBOT_STOPPED } RobotState;
typedef enum { ROBOT_STEP_STOPPED, ROBOT_STEP_AUTO, ROBOT_STEP_MANUAL } RobotStepMode;
typedef struct {
    RobotState state;
    RobotStepMode step_mode;
    uint32_t speed;
    uint8_t target, fault;
    int duty;
    int servo_angle[2]; /* -1 until explicitly commanded */
    int servo_pulse_us[2]; /* Exact target width; -1 until commanded. */
    uint32_t demo_position; /* STEP pulses from SW4; not encoder feedback. */
    uint8_t demo_homing;
    uint32_t outward_steps, return_steps; /* Last outward endpoint and current return count. */
    int32_t home_step_error; /* Outward minus return pulses at the last home trigger. */
    uint32_t turn_number, turn_position;
    unsigned turn_pulse_us;
    uint8_t turn_switch;
} RobotStatus;

void Robot_Init(TIM_HandleTypeDef *motor, TIM_HandleTypeDef *stepper, TIM_HandleTypeDef *servos);
void Robot_Update(void);
void Robot_Stop(void);
const RobotStatus *Robot_GetStatus(void);
const char *Robot_StartAuto(void);
const char *Robot_StepperAuto(void);
const char *Robot_Move(int32_t steps);
const char *Robot_SetSpeed(uint32_t speed);
const char *Robot_SetMotor(int duty);
const char *Robot_SetServo(unsigned id, unsigned angle);
const char *Robot_SetServoPulseUs(unsigned id, unsigned pulse_us);
void Robot_StopStepper(void);
void Robot_StopMotor(void);
#endif
