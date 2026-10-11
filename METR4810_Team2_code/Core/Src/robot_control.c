#include "robot_control.h"
#include "motor.h"
#include "stepper.h"
#include "servo.h"
#include "switch.h"

#define DIR_TO_SWITCH4 0U
#define SWITCH_DEBOUNCE_MS 30U
static RobotStatus status;
static uint8_t debouncing;
static uint32_t debounce_start;
static uint8_t button_raw, button_stable;
static uint32_t button_changed;
static void start_auto(void);

static uint8_t run_button_pressed(void)
{
    uint8_t pressed = Switch_IsPressed(2);
    uint32_t now = HAL_GetTick();
    if (pressed != button_raw) {
        button_raw = pressed;
        button_changed = now;
    }
    if (button_stable != button_raw &&
        (uint32_t)(now - button_changed) >= SWITCH_DEBOUNCE_MS) {
        button_stable = button_raw;
        return button_stable; /* One event per press; release must also debounce. */
    }
    return 0;
}

static void start_target(void)
{
    Stepper1_Start(status.target == 4U ? DIR_TO_SWITCH4 : (DIR_TO_SWITCH4 ^ 1U));
}

void Robot_Init(TIM_HandleTypeDef *motor, TIM_HandleTypeDef *stepper, TIM_HandleTypeDef *servos)
{
    status.state = ROBOT_IDLE;
    status.step_mode = ROBOT_STEP_STOPPED;
    status.speed = 500U;
    status.target = 4U;
    status.duty = status.fault = 0;
    status.servo_angle[0] = status.servo_angle[1] = -1;
    Motor1_Init(motor); /* Driver starts PWM at zero duty. */
    Stepper1_Init(stepper);
    Stepper1_SetSpeed(status.speed);
    Servo_Init(servos); /* Initializes targets without enabling PWM. */
    debouncing = 0;
    /* Holding the button during reset must not start motion. */
    button_raw = button_stable = Switch_IsPressed(2);
    button_changed = HAL_GetTick();
    Robot_Update();
}

void Robot_Stop(void)
{
    Stepper1_Stop();
    Motor1_SetDuty(0);
    Servo_Stop();
    status.duty = 0;
    status.step_mode = ROBOT_STEP_STOPPED;
    status.state = ROBOT_STOPPED;
    debouncing = 0;
}

void Robot_Update(void)
{
    uint8_t toggle = run_button_pressed();
    if (Switch_IsPressed(1) && Switch_IsPressed(4)) status.fault = 1;
    if (status.fault) {
        Robot_Stop();
        return;
    }
    if (toggle) {
        if (status.state == ROBOT_RUNNING) Robot_Stop();
        else start_auto();
    }
    if (status.step_mode == ROBOT_STEP_MANUAL) {
        if (Switch_IsPressed(status.target) || !Stepper1_IsBusy()) {
            Stepper1_Stop();
            status.step_mode = ROBOT_STEP_STOPPED;
        }
    } else if (status.step_mode == ROBOT_STEP_AUTO) {
        if (debouncing) {
            if ((uint32_t)(HAL_GetTick() - debounce_start) >= SWITCH_DEBOUNCE_MS) {
                if (Switch_IsPressed(status.target))
                    status.target = status.target == 4U ? 1U : 4U;
                /* A false press resumes toward the same target. */
                debouncing = 0;
                start_target();
            }
        } else if (Switch_IsPressed(status.target)) {
            Stepper1_Stop();
            debouncing = 1;
            debounce_start = HAL_GetTick();
        }
    }
}

static const char *motion_allowed(uint8_t individual)
{
    Robot_Update();
    if (status.fault) return "LIMIT FAULT LATCHED: RESET REQUIRED";
    if (individual && status.state == ROBOT_RUNNING) return "AUTONOMOUS ACTIVE: send stop first";
    return NULL;
}

static void begin_auto(void)
{
    Stepper1_Stop();
    debouncing = 0;
    status.target = Switch_IsPressed(4) ? 1U : 4U;
    status.step_mode = ROBOT_STEP_AUTO;
    start_target();
}

static void start_auto(void)
{
    Servo_Stop();
    begin_auto();
    Motor1_SetDuty(100);
    status.duty = 100;
    status.state = ROBOT_RUNNING;
}

const char *Robot_StartAuto(void)
{
    const char *error = motion_allowed(0);
    if (error) return error;
    if (status.state != ROBOT_RUNNING) start_auto();
    return NULL;
}

const char *Robot_StepperAuto(void)
{
    const char *error = motion_allowed(1);
    if (error) return error;
    begin_auto();
    status.state = ROBOT_IDLE;
    return NULL;
}

const char *Robot_Move(int32_t steps)
{
    if (!steps || steps < -1000000 || steps > 1000000) return "STEPS MUST BE -1000000..-1 OR 1..1000000";
    const char *error = motion_allowed(1);
    if (error) return error;
    uint8_t target = steps > 0 ? 4U : 1U;
    if (Switch_IsPressed(target)) return "TARGET LIMIT ACTIVE";
    if (status.step_mode != ROBOT_STEP_STOPPED) return "STEPPER BUSY: send stepper(stop) first";
    status.target = target;
    status.step_mode = ROBOT_STEP_MANUAL;
    status.state = ROBOT_IDLE;
    Stepper1_Move(target == 4U ? DIR_TO_SWITCH4 : (DIR_TO_SWITCH4 ^ 1U),
                  (uint32_t)(steps < 0 ? -steps : steps));
    return NULL;
}

const char *Robot_SetSpeed(uint32_t speed)
{
    if (speed < 16 || speed > 2000) return "INVALID SPEED: USE 16..2000 STEPS/S";
    if (status.state == ROBOT_RUNNING || status.step_mode != ROBOT_STEP_STOPPED)
        return "STOP STEPPER BEFORE CHANGING SPEED";
    status.speed = speed;
    Stepper1_SetSpeed(speed);
    return NULL;
}

const char *Robot_SetMotor(int duty)
{
    if (duty < -100 || duty > 100) return "DUTY OUT OF RANGE: USE -100..100";
    if (!duty) { Robot_StopMotor(); return NULL; }
    const char *error = motion_allowed(1);
    if (error) return error;
    Motor1_SetDuty((int)duty);
    status.duty = duty;
    status.state = ROBOT_IDLE;
    return NULL;
}

const char *Robot_SetServo(unsigned id, unsigned angle)
{
    if (id < 1 || id > 2) return "INVALID SERVO: USE 1 OR 2";
    if (angle > SERVO_MAX_ANGLE) return "ANGLE OUT OF RANGE: USE 0..120";
    const char *error = motion_allowed(1);
    if (error) return error;
    if (servo((uint8_t)id, (int)angle) != HAL_OK || Servo_Start((uint8_t)id) != HAL_OK)
        return "SERVO DRIVER ERROR";
    status.servo_angle[id - 1] = (int)angle;
    status.state = ROBOT_IDLE;
    return NULL;
}

void Robot_StopStepper(void)
{
    if (status.state == ROBOT_RUNNING) { Robot_Stop(); return; }
    Stepper1_Stop();
    status.step_mode = ROBOT_STEP_STOPPED;
    debouncing = 0;
}

void Robot_StopMotor(void)
{
    if (status.state == ROBOT_RUNNING) { Robot_Stop(); return; }
    Motor1_SetDuty(0);
    status.duty = 0;
}

const RobotStatus *Robot_GetStatus(void) { return &status; }
