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
static uint32_t arc_test_start;
static uint32_t debounce_start;
static uint8_t run_switch_raw, run_switch_stable;
static uint32_t run_switch_changed;
static void start_auto(void);
static void arc_test_update(void);
static void finish_run(void);
static void turn_at_outward_end(void);
static uint32_t demo_leg_start_pulses, demo_leg_start_position;

/* Foam-clearance path, 24 Oct 2026. Anchors are servo degrees at DATA positions
   measured from SW4 first touch (with overtravel disabled). The final anchor is
   the counted outward reversal position, 5000 steps before the measured SW1 hit.
   Monotone cubic Hermite interpolation: weighted harmonic interior slopes,
   zero slope at the endpoints and on both sides of the 100-degree plateau.
   Slopes are degrees per 1000 steps. The return uses this exact same curve. */
static const struct { uint32_t step; float angle, slope; } demo_schedule[] = {
    {100000U, 90.0f, 0.0f},
    {105000U, 100.0f, 0.0f},
    {115000U, 100.0f, 0.0f},
    {125000U, 110.0f, 0.75f},
    {135000U, 116.0f, 0.6f},
    {155000U, 128.0f, 1.219093498f},
    {RUN_OUTWARD_END_STEPS, 135.0f, 0.0f}
};
#define DEMO_SCHEDULE_ROWS (sizeof demo_schedule / sizeof demo_schedule[0])

/* Preserve the physical path if zero moves behind SW4 for limit overtravel. */
static uint32_t path_step(uint32_t position)
{
    return position > LIMIT_OVERTRAVEL_STEPS ? position - LIMIT_OVERTRAVEL_STEPS : 0U;
}

static unsigned demo_pulse(uint32_t step)
{
    if (step <= demo_schedule[0].step) return DEMO_START_PULSE_US;
    for (unsigned i = 1; i < DEMO_SCHEDULE_ROWS; ++i) {
        if (step <= demo_schedule[i].step) {
            float span = (float)(demo_schedule[i].step - demo_schedule[i - 1].step);
            float t = (float)(step - demo_schedule[i - 1].step) / span;
            float t2 = t * t, t3 = t2 * t;
            float angle = (2.0f*t3 - 3.0f*t2 + 1.0f) * demo_schedule[i - 1].angle +
                (t3 - 2.0f*t2 + t) * (span / 1000.0f) * demo_schedule[i - 1].slope +
                (-2.0f*t3 + 3.0f*t2) * demo_schedule[i].angle +
                (t3 - t2) * (span / 1000.0f) * demo_schedule[i].slope;
            /* Bound round-off locally as well as protecting the overall arc. */
            if (angle < demo_schedule[i - 1].angle) angle = demo_schedule[i - 1].angle;
            if (angle > demo_schedule[i].angle) angle = demo_schedule[i].angle;
            unsigned pulse = (unsigned)(SERVO_MIN_PULSE_US + angle *
                (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / SERVO_MAX_ANGLE + 0.5f);
            if (pulse < DEMO_START_PULSE_US) return DEMO_START_PULSE_US;
            return pulse > DEMO_END_PULSE_US ? DEMO_END_PULSE_US : pulse;
        }
    }
    return DEMO_END_PULSE_US; /* Clamp at the final path angle. */
}

/* Command servo 2 only when the pulse changes, and keep the reported angle in step. */
static void demo_servo(unsigned pulse)
{
    pulse = (unsigned)((int)pulse + (int)SERVO_ANGLE_US(status.straight_angle) - (int)DEMO_START_PULSE_US);
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
    demo_servo(demo_pulse(path_step(status.demo_position)));
}

/* PC11 maintained run switch: report either debounced edge once. */
static uint8_t run_switch_changed_state(void)
{
    uint8_t pressed = Switch_IsPressed(2);
    uint32_t now = HAL_GetTick();
    if (pressed != run_switch_raw) {
        run_switch_raw = pressed;
        run_switch_changed = now;
    }
    if (run_switch_stable != run_switch_raw &&
        (uint32_t)(now - run_switch_changed) >= SWITCH_DEBOUNCE_MS) {
        run_switch_stable = run_switch_raw;
        return 1; /* Both ON and OFF transitions must debounce. */
    }
    return 0;
}

/* DIR level that drives toward the current target switch. */
static uint8_t target_direction(void)
{
    return status.target == 4U ? DIR_TO_SWITCH4 : (DIR_TO_SWITCH4 ^ 1U);
}

static uint32_t run_speed(void)
{
    if (status.demo_homing) return status.straight_speed;
    if (status.target == 4U) return status.return_speed;
    return path_step(status.demo_position) >= demo_schedule[0].step ?
        status.curve_speed : status.straight_speed;
}

static void start_target(void)
{
    if (status.state == ROBOT_RUNNING) {
        status.speed = run_speed();
        Stepper1_SetSpeed(status.speed);
    }
    if (status.state == ROBOT_RUNNING && !status.demo_homing && status.target == 1U) {
        if (status.demo_position >= RUN_OUTWARD_STOP_POSITION) turn_at_outward_end();
        else Stepper1_Move(target_direction(), RUN_OUTWARD_STOP_POSITION - status.demo_position);
    } else Stepper1_Start(target_direction());
}

/* The driver changes speed only while stopped. Account for an in-flight pulse
   before restarting, so the servo and return reference keep the same position. */
static void update_run_speed(void)
{
    if (status.state != ROBOT_RUNNING || status.paused || status.demo_homing ||
        debouncing || overtravel || !Stepper1_IsBusy() || status.speed == run_speed()) return;
    Stepper1_Stop();
    demo_update_position();
    start_target();
}

/* Reverse at the target limit switch, toward the other one. */
static void turn_around(void)
{
    if (status.state == ROBOT_RUNNING) {
        status.turn_at_position = 0;
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
            demo_servo(DEMO_START_PULSE_US);
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

static void turn_at_outward_end(void)
{
    turn_around();
    status.turn_at_position = 1;
}

/* Physical SW4 confirmed: stop at first touch, without return overtravel. */
static void finish_run(void)
{
    status.demo_position = RUN_HOME_POSITION;
    demo_servo(DEMO_START_PULSE_US);
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
        int64_t error = (int64_t)status.outward_steps - status.return_steps - RUN_HOME_POSITION;
        status.home_step_error = error > INT32_MAX ? INT32_MAX :
            error < INT32_MIN ? INT32_MIN : (int32_t)error;
        finish_run();
        return;
    }
    if (status.state == ROBOT_RUNNING && !status.demo_homing && status.target == 1U) {
        uint32_t left = status.demo_position < RUN_OUTWARD_STOP_POSITION ?
            RUN_OUTWARD_STOP_POSITION - status.demo_position : 0U;
        if (steps > left) steps = left; /* Never overtravel beyond the counted outward end. */
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
    status.straight_speed = 1000U;
    status.curve_speed = 500U;
    status.return_speed = 1500U;
    status.speed = status.straight_speed;
    status.target = 4U;
    status.duty = status.fault = 0;
    status.straight_angle = DEMO_START_ANGLE;
    status.servo_angle[0] = status.servo_angle[1] = -1;
    status.servo_pulse_us[0] = status.servo_pulse_us[1] = -1;
    status.demo_position = 0;
    status.demo_homing = status.demo_backoff = 0;
    status.outward_steps = status.return_steps = 0;
    status.home_step_error = 0;
    status.turn_number = status.turn_position = status.turn_pulse_us = 0;
    status.turn_switch = status.turn_at_position = 0;
    status.run_complete = status.paused = 0;
    status.pause_remaining = 0;
    status.arc_test = 0;
    status.arc_tests_done = 0;
    Motor1_Init(motor); /* Driver starts PWM at zero duty. */
    Stepper1_Init(stepper);
    Stepper1_SetSpeed(status.speed);
    Servo_Init(servos); /* Initializes targets without enabling PWM. */
    debouncing = overtravel = 0;
    /* An ON switch at reset must go OFF then ON before it can start motion. */
    run_switch_raw = run_switch_stable = Switch_IsPressed(2);
    run_switch_changed = HAL_GetTick();
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
    status.arc_test = 0;
}

void Robot_Update(void)
{
    uint8_t switch_changed = run_switch_changed_state();
    if (!status.fault && Switch_IsPressed(1) && Switch_IsPressed(4)) status.fault = ROBOT_FAULT_BOTH_LIMITS;
    if (status.fault) {
        Robot_Stop();
        return;
    }
    if (switch_changed) {
        if (!run_switch_stable) Robot_Stop();
        else if (status.state != ROBOT_RUNNING) start_auto();
    }
    if (status.state == ROBOT_RUNNING) demo_update_position();
    if (status.state == ROBOT_RUNNING && status.paused) return; /* Frozen until continue or stop. */
    if (status.arc_test) arc_test_update();
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
    if (status.state == ROBOT_RUNNING && !status.demo_homing && status.target == 1U &&
        !debouncing && !overtravel && !Stepper1_IsBusy()) turn_at_outward_end();
    update_run_speed();
}

/* servo(arc): ramp out to the end of the arc, hold, ramp back, then stay at the arc start. */
static void arc_test_update(void)
{
    uint32_t t = HAL_GetTick() - arc_test_start;
    uint32_t span = DEMO_END_PULSE_US - DEMO_START_PULSE_US;
    unsigned pulse = DEMO_START_PULSE_US;
    if (t < ARC_TEST_SWEEP_MS)
        pulse = DEMO_START_PULSE_US + (t * span + ARC_TEST_SWEEP_MS / 2U) / ARC_TEST_SWEEP_MS;
    else if (t < ARC_TEST_SWEEP_MS + ARC_TEST_HOLD_MS)
        pulse = DEMO_END_PULSE_US;
    else if (t < 2U * ARC_TEST_SWEEP_MS + ARC_TEST_HOLD_MS)
        pulse = DEMO_END_PULSE_US - ((t - ARC_TEST_SWEEP_MS - ARC_TEST_HOLD_MS) * span +
                                     ARC_TEST_SWEEP_MS / 2U) / ARC_TEST_SWEEP_MS;
    else {
        status.arc_test = 0;
        ++status.arc_tests_done;
    }
    demo_servo(pulse);
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

/* Start a run: DC off, servo at the arc start, then home to SW4 (backing off it first if pressed). */
static void start_auto(void)
{
    Stepper1_Stop();
    status.speed = status.straight_speed;
    Stepper1_SetSpeed(status.speed);
    Motor1_SetDuty(0);
    /* Re-home on every run; never treat an arbitrary starting point as zero.
       A running servo PWM takes the new width at its next period, without a glitch. */
    status.servo_pulse_us[1] = -1;
    demo_servo(DEMO_START_PULSE_US);
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
    status.arc_test = 0;
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

const char *Robot_ServoArcTest(void)
{
    const char *error = motion_allowed(1);
    if (error) return error;
    demo_servo(DEMO_START_PULSE_US);
    if (Servo_Start(2U) != HAL_OK) return "SERVO DRIVER ERROR";
    arc_test_start = HAL_GetTick();
    status.arc_test = 1U;
    status.state = ROBOT_IDLE;
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

const char *Robot_SetRunSpeeds(uint32_t straight, uint32_t curve, uint32_t reverse)
{
    if (straight < 16 || straight > 2000 || curve < 16 || curve > 2000 ||
        reverse < 16 || reverse > 2000) return "INVALID SPEED: USE 16..2000 STEPS/S";
    if (status.state == ROBOT_RUNNING || status.step_mode != ROBOT_STEP_STOPPED)
        return "STOP STEPPER BEFORE CHANGING SPEED";
    status.straight_speed = straight;
    status.curve_speed = curve;
    status.return_speed = reverse;
    status.speed = straight;
    Stepper1_SetSpeed(straight);
    return NULL;
}

/* Legacy single-speed command selects a uniform profile and manual speed. */
const char *Robot_SetSpeed(uint32_t speed)
{
    return Robot_SetRunSpeeds(speed, speed, speed);
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

const char *Robot_Calibrate(unsigned angle)
{
    if (angle > SERVO_MAX_ANGLE - (DEMO_END_ANGLE - DEMO_START_ANGLE))
        return "CALIBRATION OUT OF RANGE: USE 0..135 (FULL SWEEP MUST FIT 0..180)";
    const char *error = motion_allowed(1);
    if (error) return error;
    if (status.step_mode != ROBOT_STEP_STOPPED || status.duty || status.arc_test)
        return "STOP ALL MOTION BEFORE CALIBRATING";
    /* Manual servo commands stay absolute, so the user can find straight first. */
    error = Robot_SetServo(2U, angle);
    if (error) return error;
    status.straight_angle = angle;
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
    status.arc_test = 0; /* A manual target ends any sweep. */
    status.servo_angle[id - 1] = (int)angle;
    status.servo_pulse_us[id - 1] = (int)SERVO_ANGLE_US(angle);
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
    status.arc_test = 0; /* A manual target ends any sweep. */
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
