#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include "stm32g4xx_hal.h"
#include "servo.h"

/* STEP pulses driven past a debounced SW1/SW4 press before reversing; the SW4
   reversal point is position zero. Keep 0 until dampened switch mounts are
   fitted, then 1500. */
#ifndef LIMIT_OVERTRAVEL_STEPS
#define LIMIT_OVERTRAVEL_STEPS 0U
#endif
/* A run that starts with SW4 pressed first moves this far toward SW1, checks SW4
   has released, then homes to first touch as normal. */
#define RUN_START_BACKOFF_STEPS (1500U + LIMIT_OVERTRAVEL_STEPS)
/* status.fault values; any non-zero fault stops everything until reset. */
#define ROBOT_FAULT_BOTH_LIMITS 1U /* SW1 and SW4 pressed together. */
#define ROBOT_FAULT_SW4_STUCK 2U /* SW4 still pressed after the run-start back-off. */
/* The run's servo arc: it starts at the servo centre (CAD offset 0) and never goes
   past DEMO_END_ANGLE. */
#define DEMO_START_PULSE_US SERVO_ANGLE_US(SERVO_CENTRE_ANGLE) /* 95 degrees: 1556 us. */
#define DEMO_END_ANGLE 135
#define DEMO_END_PULSE_US SERVO_ANGLE_US(DEMO_END_ANGLE) /* 2000 us. */
/* servo(arc) timing: sweep out, hold at the end, sweep back. */
#define ARC_TEST_SWEEP_MS 4000U
#define ARC_TEST_HOLD_MS 1000U
#define DEMO_US_PER_POD_DEG 20U /* Nominal scale through the 1.8:1 gears. */

/* RUNNING means a demo run is active (possibly paused). IDLE covers manual CLI use. */
typedef enum { ROBOT_IDLE, ROBOT_RUNNING, ROBOT_STOPPED } RobotState;
typedef enum { ROBOT_STEP_STOPPED, ROBOT_STEP_AUTO, ROBOT_STEP_MANUAL } RobotStepMode;
/* Everything the CLI reports. Read-only outside robot_control.c. */
typedef struct {
    RobotState state;
    RobotStepMode step_mode;
    uint32_t speed; /* Stepper speed in STEP pulses per second. */
    uint8_t target, fault; /* target: switch being driven toward (1 or 4); fault: ROBOT_FAULT_* or 0. */
    int duty; /* DC drill duty, -100..100 percent. */
    int servo_angle[2]; /* -1 until explicitly commanded */
    int servo_pulse_us[2]; /* Exact target width; -1 until commanded. */
    uint32_t demo_position; /* STEP pulses from the SW4 reversal point; not encoder feedback. */
    uint8_t demo_homing; /* 1 until SW4 first touch sets zero. */
    uint8_t demo_backoff; /* 1 while a run that started on SW4 backs off before homing. */
    uint32_t outward_steps, return_steps; /* Last outward endpoint and current return count. */
    int32_t home_step_error; /* Position at the last SW4 first touch minus LIMIT_OVERTRAVEL_STEPS. */
    uint32_t turn_number, turn_position;
    unsigned turn_pulse_us;
    uint8_t turn_switch; /* Limit switch (1 or 4) of the last demo turn. */
    uint8_t run_complete; /* 1 once a run has finished its cycle back at SW4. */
    uint8_t paused; /* 1 while a run is frozen, waiting for continue or stop. */
    uint32_t pause_remaining; /* Back-off or overtravel pulses that continue will finish. */
    uint8_t arc_test; /* 1 while servo(arc) is sweeping. */
    uint32_t arc_tests_done; /* Sweeps that finished back at the centre. */
} RobotStatus;

/* Start the drivers with the stepper and DC motor off and no servo signal. Call once after the MX inits. */
void Robot_Init(TIM_HandleTypeDef *motor, TIM_HandleTypeDef *stepper, TIM_HandleTypeDef *servos);
/* Check limits, faults and the PC11 button, then advance any run. Call on every main-loop pass. */
void Robot_Update(void);
/* Stop the stepper and DC motor where they are and end any run. The servo keeps its angle. */
void Robot_Stop(void);
const RobotStatus *Robot_GetStatus(void);

/* The commands below return NULL on success, or a short error message for the CLI. */

/* One demo cycle: home to SW4, drive out to SW1, return to SW4 and stop. */
const char *Robot_StartAuto(void);
/* Freeze a run in place without ending it. */
const char *Robot_Pause(void);
/* Resume a paused run on the same leg, from where it stopped. */
const char *Robot_Continue(void);
/* Servo 2 sweeps the run's arc (centre, end, centre) with nothing else moving. */
const char *Robot_ServoArcTest(void);
/* Stepper-only test: travel between SW1 and SW4 until stopped. Leaves the DC motor alone. */
const char *Robot_StepperAuto(void);
/* Counted move; positive steps go toward SW4. Stops early at the target switch. */
const char *Robot_Move(int32_t steps);
/* 16..2000 pulses per second. The stepper must be stopped. */
const char *Robot_SetSpeed(uint32_t speed);
/* Signed duty, -100..100 percent. 0 stops the motor. */
const char *Robot_SetMotor(int duty);
/* Servo target as a nominal angle (0..180) or a pulse width (500..2500 us). Starts that servo's PWM. */
const char *Robot_SetServo(unsigned id, unsigned angle);
const char *Robot_SetServoPulseUs(unsigned id, unsigned pulse_us);
/* Stop one actuator. During a run, either of these ends the whole run. */
void Robot_StopStepper(void);
void Robot_StopMotor(void);
#endif
