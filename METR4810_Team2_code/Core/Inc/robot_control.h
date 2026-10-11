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
void Robot_StopStepper(void);
void Robot_StopMotor(void);
#endif
