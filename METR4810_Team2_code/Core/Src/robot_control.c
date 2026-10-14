#include "robot_control.h"
#include "main.h"
#include "motor.h"
#include "stepper.h"
#include "servo.h"
#include "switch.h"

#define DIR_TO_SWITCH4 0U
#define SWITCH_DEBOUNCE_MS 30U
#define DEMO_OUTWARD_DC_DUTY (-100)
#define DEMO_RETURN_DC_DUTY 100
static RobotStatus status;
static uint8_t debouncing;
static uint32_t debounce_start;
static uint8_t button_raw, button_stable;
static uint32_t button_changed;
static void start_auto(void);
static uint32_t demo_leg_start_pulses, demo_leg_start_position;

/* User-supplied position schedule: SW4 = 0, outward travel toward SW1.
   Interpolate pulse widths, not the rounded angles shown in the guide. */
static const struct { uint32_t step; unsigned pulse; } demo_schedule[] = {
    {0U, 1500U}, {59200U, 1500U}, {70000U, 1505U},
    {80000U, 1520U}, {90000U, 1544U}, {100000U, 1578U},
    {110000U, 1621U}, {120000U, 1673U}, {130000U, 1734U},
    {131800U, 1745U}, {140000U, 1800U}, {145000U, 1833U}
};

static unsigned demo_pulse(uint32_t position)
{
    for (unsigned i = 1; i < sizeof demo_schedule / sizeof demo_schedule[0]; ++i) {
        if (position <= demo_schedule[i].step) {
            uint32_t span = demo_schedule[i].step - demo_schedule[i - 1].step;
            uint32_t offset = position - demo_schedule[i - 1].step;
            return demo_schedule[i - 1].pulse +
                (offset * (demo_schedule[i].pulse - demo_schedule[i - 1].pulse) + span / 2U) / span;
        }
    }
    return 1833U; /* Hold final angle if the physical limit is beyond the table. */
}

static void demo_servo(unsigned pulse)
{
    if (status.servo_pulse_us[1] == (int)pulse) return;
    if (Servo_SetPulseUs(2U, pulse) != HAL_OK) { Error_Handler(); return; }
    status.servo_pulse_us[1] = (int)pulse;
    status.servo_angle[1] = ((pulse - SERVO_MIN_PULSE_US) * SERVO_MAX_ANGLE + 1000U) / 2000U;
}

static void demo_update_position(void)
{
    uint32_t total = Stepper1_TotalPulses();
    uint32_t delta = total - demo_leg_start_pulses;
    if (status.demo_homing) return;
    if (status.target == 1U) {
        status.demo_position = delta > UINT32_MAX - demo_leg_start_position ? UINT32_MAX : demo_leg_start_position + delta;
    } else {
        /* Retrace from the ACTUAL turnaround, never from a hard-coded 145000.
           Keep a fixed reference for the whole return, including switch bounce stops. */
        status.return_steps = delta;
        status.demo_position = delta > demo_leg_start_position ? 0U : demo_leg_start_position - delta;
    }
    demo_servo(demo_pulse(status.demo_position));
}

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
    status.servo_pulse_us[0] = status.servo_pulse_us[1] = -1;
    status.demo_position = 0;
    status.demo_homing = 0;
    status.outward_steps = status.return_steps = 0;
    status.home_step_error = 0;
    status.turn_number = status.turn_position = status.turn_pulse_us = 0;
    status.turn_switch = 0;
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
    status.demo_homing = 0;
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
    if (status.state == ROBOT_RUNNING) demo_update_position();
    if (status.step_mode == ROBOT_STEP_MANUAL) {
        if (Switch_IsPressed(status.target) || !Stepper1_IsBusy()) {
            Stepper1_Stop();
            status.step_mode = ROBOT_STEP_STOPPED;
        }
    } else if (status.step_mode == ROBOT_STEP_AUTO) {
        if (debouncing) {
            if ((uint32_t)(HAL_GetTick() - debounce_start) >= SWITCH_DEBOUNCE_MS) {
                if (Switch_IsPressed(status.target)) {
                    if (status.state == ROBOT_RUNNING) {
                        status.turn_switch = status.target;
                        status.turn_position = status.demo_position;
                        status.turn_pulse_us = (unsigned)status.servo_pulse_us[1];
                        ++status.turn_number;
                        if (status.target == 1U) {
                            status.outward_steps = status.demo_position;
                            status.return_steps = 0;
                        } else if (!status.demo_homing) {
                            int64_t error = (int64_t)status.outward_steps - status.return_steps;
                            status.home_step_error = error > INT32_MAX ? INT32_MAX :
                                error < INT32_MIN ? INT32_MIN : (int32_t)error;
                        }
                    }
                    if (status.state == ROBOT_RUNNING && status.target == 4U) {
                        status.demo_position = 0U;
                        status.demo_homing = 0U;
                        demo_servo(1500U);
                    }
                    status.target = status.target == 4U ? 1U : 4U;
                    if (status.state == ROBOT_RUNNING) {
                        demo_leg_start_position = status.demo_position;
                        demo_leg_start_pulses = Stepper1_TotalPulses();
                        status.duty = status.target == 1U ? DEMO_OUTWARD_DC_DUTY : DEMO_RETURN_DC_DUTY;
                        Motor1_SetDuty(status.duty);
                    }
                }
                /* A false press resumes toward the same target. */
                debouncing = 0;
                start_target();
            }
        } else if (Switch_IsPressed(status.target)) {
            Stepper1_Stop();
            if (status.state == ROBOT_RUNNING) demo_update_position();
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
    Stepper1_Stop();
    Motor1_SetDuty(0);
    Servo_Stop();
    /* Re-home on every run; never treat an arbitrary starting point as zero. */
    status.servo_pulse_us[1] = -1;
    demo_servo(1500U);
    if (Servo_Start(2U) != HAL_OK) { Error_Handler(); return; }
    status.duty = 0;
    status.demo_homing = 1U;
    status.demo_position = 0U;
    demo_leg_start_pulses = Stepper1_TotalPulses();
    demo_leg_start_position = 0;
    status.outward_steps = status.return_steps = 0;
    status.home_step_error = 0;
    status.target = 4U;
    status.step_mode = ROBOT_STEP_AUTO;
    status.state = ROBOT_RUNNING;
    debouncing = Switch_IsPressed(4);
    debounce_start = HAL_GetTick();
    if (!debouncing) start_target();
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
    if (angle > SERVO_MAX_ANGLE) return "ANGLE OUT OF RANGE: USE 0..180";
    const char *error = motion_allowed(1);
    if (error) return error;
    if (servo((uint8_t)id, (int)angle) != HAL_OK || Servo_Start((uint8_t)id) != HAL_OK)
        return "SERVO DRIVER ERROR";
    status.servo_angle[id - 1] = (int)angle;
    status.servo_pulse_us[id - 1] = SERVO_MIN_PULSE_US +
        (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) + SERVO_MAX_ANGLE / 2) / SERVO_MAX_ANGLE;
    status.state = ROBOT_IDLE;
    return NULL;
}

const char *Robot_SetServoPulseUs(unsigned id, unsigned pulse_us)
{
    if (id < 1 || id > 2) return "INVALID SERVO: USE 1 OR 2";
    if (pulse_us < SERVO_MIN_PULSE_US || pulse_us > SERVO_MAX_PULSE_US)
        return "PULSE OUT OF RANGE: USE 500..2500 US";
    const char *error = motion_allowed(1);
    if (error) return error;
    if (Servo_SetPulseUs((uint8_t)id, pulse_us) != HAL_OK || Servo_Start((uint8_t)id) != HAL_OK)
        return "SERVO DRIVER ERROR";
    status.servo_pulse_us[id - 1] = (int)pulse_us;
    status.servo_angle[id - 1] = ((pulse_us - SERVO_MIN_PULSE_US) * SERVO_MAX_ANGLE +
        (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 2) / (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
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
