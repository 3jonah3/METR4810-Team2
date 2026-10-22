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
static uint8_t debouncing, overtravel;
static uint32_t debounce_start;
static uint8_t button_raw, button_stable;
static uint32_t button_changed;
static void start_auto(void);
static uint32_t demo_leg_start_pulses, demo_leg_start_position;

/* CAD steering schedule (Steering_schedule_from_CAD.csv, 21 Oct 2026): servo offset
   from straight that keeps the cutter tip on the bore centreline, at the nominal
   DEMO_US_PER_POD_DEG scale. CSV steps count from the CAD zero,
   the rear reversal point 1500 steps past SW4 first touch (398.87 steps/mm).
   The run follows it out to SW1 (CSV 160250 at first touch).
   Interpolate pulse widths, not the rounded angles shown in the guide. */
#define CAD_ZERO_PAST_SW4 1500U
/* The pod keeper pocket allows 23.5 degrees but the last rows need 23.6-24.1, so
   the offset is capped just inside it (23.4 degrees) for the final ~1.6 mm. */
#define DEMO_MAX_OFFSET_US 468U
static const struct { uint32_t step; unsigned offset; } demo_schedule[] = {
    {0U, 0U}, {66715U, 0U}, {69706U, 3U}, {73695U, 7U},
    {77684U, 14U}, {81672U, 22U}, {85661U, 32U}, {89650U, 44U},
    {93638U, 57U}, {97627U, 72U}, {101616U, 88U}, {105604U, 107U},
    {109593U, 127U}, {113582U, 148U}, {117570U, 171U}, {121559U, 196U},
    {125548U, 223U}, {129537U, 251U}, {133525U, 280U}, {137514U, 308U},
    {141503U, 337U}, {144853U, 361U}, {145491U, 366U}, {149480U, 394U},
    {153469U, 423U}, {157457U, 452U}, {160250U, 472U}, {161750U, 483U}
};
#define DEMO_SCHEDULE_ROWS (sizeof demo_schedule / sizeof demo_schedule[0])

/* Demo position (pulses from the SW4 reversal point) in CSV steps. */
static uint32_t cad_step(uint32_t position)
{
    uint64_t step = (uint64_t)position + CAD_ZERO_PAST_SW4;
    step = step > LIMIT_OVERTRAVEL_STEPS ? step - LIMIT_OVERTRAVEL_STEPS : 0U;
    return step > UINT32_MAX ? UINT32_MAX : (uint32_t)step;
}

/* Servo pulse for a CSV step: linear between rows, held after the last row, and never
   past the keeper cap. */
static unsigned demo_pulse(uint32_t step)
{
    /* Hold the final angle if travel continues beyond the table. */
    unsigned offset = demo_schedule[DEMO_SCHEDULE_ROWS - 1U].offset;
    for (unsigned i = 1; i < DEMO_SCHEDULE_ROWS; ++i) {
        if (step <= demo_schedule[i].step) {
            uint32_t span = demo_schedule[i].step - demo_schedule[i - 1].step;
            uint32_t into = step - demo_schedule[i - 1].step;
            offset = demo_schedule[i - 1].offset +
                (into * (demo_schedule[i].offset - demo_schedule[i - 1].offset) + span / 2U) / span;
            break;
        }
    }
    return DEMO_STRAIGHT_PULSE_US + (offset > DEMO_MAX_OFFSET_US ? DEMO_MAX_OFFSET_US : offset);
}

/* Command servo 2 only when the pulse changes, and keep the reported angle in step. */
static void demo_servo(unsigned pulse)
{
    if (status.servo_pulse_us[1] == (int)pulse) return;
    if (Servo_SetPulseUs(2U, pulse) != HAL_OK) { Error_Handler(); return; }
    status.servo_pulse_us[1] = (int)pulse;
    status.servo_angle[1] = ((pulse - SERVO_MIN_PULSE_US) * SERVO_MAX_ANGLE + 1000U) / 2000U;
}

/* Recount the position on the current leg from emitted pulses and move the servo to match. */
static void demo_update_position(void)
{
    uint32_t total = Stepper1_TotalPulses();
    uint32_t delta = total - demo_leg_start_pulses;
    if (status.demo_homing) return;
    if (status.target == 1U) {
        status.demo_position = delta > UINT32_MAX - demo_leg_start_position ? UINT32_MAX : demo_leg_start_position + delta;
    } else {
        /* Retrace from the ACTUAL turnaround, never from a hard-coded endpoint.
           Keep a fixed reference for the whole return, including switch bounce stops. */
        status.return_steps = delta;
        status.demo_position = delta > demo_leg_start_position ? 0U : demo_leg_start_position - delta;
    }
    demo_servo(demo_pulse(cad_step(status.demo_position)));
}

/* PC11 run button: returns 1 once per debounced press. */
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

/* DIR level that drives toward the current target switch. */
static uint8_t target_direction(void)
{
    return status.target == 4U ? DIR_TO_SWITCH4 : (DIR_TO_SWITCH4 ^ 1U);
}

static void start_target(void)
{
    Stepper1_Start(target_direction());
}

/* Reverse at the target limit switch, toward the other one. */
static void turn_around(void)
{
    if (status.state == ROBOT_RUNNING) {
        status.turn_switch = status.target;
        status.turn_position = status.demo_position;
        status.turn_pulse_us = (unsigned)status.servo_pulse_us[1];
        ++status.turn_number;
        if (status.target == 1U) {
            status.outward_steps = status.demo_position;
            status.return_steps = 0;
        } else {
            status.demo_position = 0U;
            status.demo_homing = 0U;
            demo_servo(DEMO_STRAIGHT_PULSE_US);
        }
    }
    status.target = status.target == 4U ? 1U : 4U;
    if (status.state == ROBOT_RUNNING) {
        demo_leg_start_position = status.demo_position;
        demo_leg_start_pulses = Stepper1_TotalPulses();
        status.duty = status.target == 1U ? DEMO_OUTWARD_DC_DUTY : DEMO_RETURN_DC_DUTY;
        Motor1_SetDuty(status.duty);
    }
    start_target();
}

/* One cycle per run: back at SW4 after SW1, stop there with the servo straight. */
static void finish_run(void)
{
    status.demo_position = 0U;
    demo_servo(DEMO_STRAIGHT_PULSE_US);
    Robot_Stop();
    status.run_complete = 1U;
}

/* The target limit is confirmed and any overtravel driven: reverse, or end the run at SW4. */
static void limit_reached(void)
{
    if (status.state == ROBOT_RUNNING && status.target == 4U && !status.demo_homing) finish_run();
    else turn_around();
}

/* The target limit stayed pressed through the debounce: drive any overtravel, then reverse. */
static void limit_confirmed(void)
{
    uint32_t steps = LIMIT_OVERTRAVEL_STEPS;
    if (status.state == ROBOT_RUNNING && status.target == 4U && !status.demo_homing) {
        /* First touch is expected LIMIT_OVERTRAVEL_STEPS before the zero point. */
        int64_t error = (int64_t)status.outward_steps - status.return_steps - LIMIT_OVERTRAVEL_STEPS;
        status.home_step_error = error > INT32_MAX ? INT32_MAX :
            error < INT32_MIN ? INT32_MIN : (int32_t)error;
    }
    if (steps) {
        overtravel = 1;
        Stepper1_Move(target_direction(), steps);
    } else limit_reached();
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
    status.demo_homing = status.demo_backoff = 0;
    status.outward_steps = status.return_steps = 0;
    status.home_step_error = 0;
    status.turn_number = status.turn_position = status.turn_pulse_us = 0;
    status.turn_switch = 0;
    status.run_complete = status.paused = 0;
    status.pause_remaining = 0;
    Motor1_Init(motor); /* Driver starts PWM at zero duty. */
    Stepper1_Init(stepper);
    Stepper1_SetSpeed(status.speed);
    Servo_Init(servos); /* Initializes targets without enabling PWM. */
    debouncing = overtravel = 0;
    /* Holding the button during reset must not start motion. */
    button_raw = button_stable = Switch_IsPressed(2);
    button_changed = HAL_GetTick();
    Robot_Update();
}

/* The servo PWM keeps running at its last target so the pod holds its angle.
   Switching it off mid-pulse can throw the servo past its limit. */
void Robot_Stop(void)
{
    Stepper1_Stop();
    Motor1_SetDuty(0);
    status.duty = 0;
    status.step_mode = ROBOT_STEP_STOPPED;
    status.state = ROBOT_STOPPED;
    debouncing = overtravel = 0;
    status.demo_homing = status.demo_backoff = status.paused = 0;
    status.pause_remaining = 0;
}

void Robot_Update(void)
{
    uint8_t toggle = run_button_pressed();
    if (!status.fault && Switch_IsPressed(1) && Switch_IsPressed(4)) status.fault = ROBOT_FAULT_BOTH_LIMITS;
    if (status.fault) {
        Robot_Stop();
        return;
    }
    if (toggle) {
        if (status.state == ROBOT_RUNNING) Robot_Stop();
        else start_auto();
    }
    if (status.state == ROBOT_RUNNING) demo_update_position();
    if (status.state == ROBOT_RUNNING && status.paused) return; /* Frozen until continue or stop. */
    if (status.step_mode == ROBOT_STEP_MANUAL) {
        if (Switch_IsPressed(status.target) || !Stepper1_IsBusy()) {
            Stepper1_Stop();
            status.step_mode = ROBOT_STEP_STOPPED;
        }
    } else if (status.step_mode == ROBOT_STEP_AUTO) {
        if (status.demo_backoff) {
            /* The run started on SW4: once the counted back-off ends, SW4 must have
               released before homing to first touch. The debounce window starts
               when the move ends. */
            if (Stepper1_IsBusy()) debounce_start = HAL_GetTick();
            else if (!Switch_IsPressed(4)) {
                status.demo_backoff = 0;
                start_target();
            } else if ((uint32_t)(HAL_GetTick() - debounce_start) >= SWITCH_DEBOUNCE_MS) {
                /* Still pressed: latch a fault rather than move further toward SW1. */
                status.fault = ROBOT_FAULT_SW4_STUCK;
                Robot_Stop();
            }
        } else if (overtravel) {
            if (!Stepper1_IsBusy()) {
                overtravel = 0;
                if (status.state == ROBOT_RUNNING) demo_update_position();
                limit_reached();
            }
        } else if (debouncing) {
            if ((uint32_t)(HAL_GetTick() - debounce_start) >= SWITCH_DEBOUNCE_MS) {
                debouncing = 0;
                if (Switch_IsPressed(status.target)) limit_confirmed();
                else start_target(); /* A false press resumes toward the same target. */
            }
        } else if (Switch_IsPressed(status.target)) {
            Stepper1_Stop();
            if (status.state == ROBOT_RUNNING) demo_update_position();
            debouncing = 1;
            debounce_start = HAL_GetTick();
        }
    }
}

/* Gate for CLI motion. A latched fault blocks everything; a run also blocks the
   individual actuator commands (individual = 1). */
static const char *motion_allowed(uint8_t individual)
{
    Robot_Update();
    if (status.fault) return "LIMIT FAULT LATCHED: RESET REQUIRED";
    if (individual && status.state == ROBOT_RUNNING) return "AUTONOMOUS ACTIVE: send stop first";
    return NULL;
}

/* stepper(auto): head for whichever end switch is not already pressed. */
static void begin_auto(void)
{
    Stepper1_Stop();
    debouncing = overtravel = 0;
    status.demo_backoff = 0;
    status.target = Switch_IsPressed(4) ? 1U : 4U;
    status.step_mode = ROBOT_STEP_AUTO;
    start_target();
}

/* Start a run: DC off, servo straight, then home to SW4 (backing off it first if pressed). */
static void start_auto(void)
{
    Stepper1_Stop();
    Motor1_SetDuty(0);
    /* Re-home on every run; never treat an arbitrary starting point as zero.
       A running servo PWM takes the new width at its next period, without a glitch. */
    status.servo_pulse_us[1] = -1;
    demo_servo(DEMO_STRAIGHT_PULSE_US);
    if (Servo_Start(2U) != HAL_OK) { Error_Handler(); return; }
    status.duty = 0;
    status.demo_homing = 1U;
    status.demo_position = 0U;
    demo_leg_start_pulses = Stepper1_TotalPulses();
    demo_leg_start_position = 0;
    status.outward_steps = status.return_steps = 0;
    status.home_step_error = 0;
    status.run_complete = status.paused = 0;
    status.pause_remaining = 0;
    status.target = 4U;
    status.step_mode = ROBOT_STEP_AUTO;
    status.state = ROBOT_RUNNING;
    debouncing = overtravel = 0;
    status.demo_backoff = 0;
    debounce_start = HAL_GetTick();
    if (Switch_IsPressed(4)) {
        /* Never zero wherever SW4 happens to be held: move off it, then home. */
        status.demo_backoff = 1U;
        Stepper1_Move(DIR_TO_SWITCH4 ^ 1U, RUN_START_BACKOFF_STEPS);
    } else start_target();
}

const char *Robot_StartAuto(void)
{
    const char *error = motion_allowed(0);
    if (error) return error;
    if (status.paused) return "RUN PAUSED: send continue or stop";
    if (status.state != ROBOT_RUNNING) start_auto();
    return NULL;
}

/* Freeze a run where it is, like stop, but keep its leg so continue can resume it. */
const char *Robot_Pause(void)
{
    const char *error = motion_allowed(0);
    if (error) return error;
    if (status.state != ROBOT_RUNNING) return "NO RUN TO PAUSE";
    if (status.paused) return NULL;
    status.pause_remaining = Stepper1_StopRemaining();
    demo_update_position();
    Motor1_SetDuty(0);
    status.duty = 0;
    status.paused = 1U;
    return NULL;
}

const char *Robot_Continue(void)
{
    const char *error = motion_allowed(0);
    if (error) return error;
    if (status.state != ROBOT_RUNNING || !status.paused) return "NOT PAUSED";
    status.paused = 0;
    status.duty = status.demo_homing ? 0 :
        status.target == 1U ? DEMO_OUTWARD_DC_DUTY : DEMO_RETURN_DC_DUTY;
    Motor1_SetDuty(status.duty);
    /* A switch being debounced must hold for a fresh 30 ms before it counts. */
    debounce_start = HAL_GetTick();
    if (debouncing) return NULL;
    if (status.demo_backoff || overtravel) {
        if (status.pause_remaining)
            Stepper1_Move(status.demo_backoff ? (DIR_TO_SWITCH4 ^ 1U) : target_direction(),
                          status.pause_remaining);
    } else start_target();
    status.pause_remaining = 0;
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
    debouncing = overtravel = 0;
}

void Robot_StopMotor(void)
{
    if (status.state == ROBOT_RUNNING) { Robot_Stop(); return; }
    Motor1_SetDuty(0);
    status.duty = 0;
}

const RobotStatus *Robot_GetStatus(void) { return &status; }
