/**
 * @file robot_control.c
 * Run/run2 state machine and safety gates. All controller state is owned by
 * the main loop; only the stepper driver counts pulses in interrupt context.
 * DEMO-prefixed names are retained for compatibility and describe the base
 * 90-135 degree curve used by both programs. No positions are encoder readings.
 */

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
/* Main-loop-owned status; other modules receive a read-only pointer. */
static RobotStatus status;
/* Target-limit debounce and the optional counted move beyond first touch. */
static uint8_t debouncing, overtravel;
static uint32_t arc_test_start;
static uint32_t debounce_start;
/* Raw and accepted PC11 states; both edges must remain stable for 30 ms. */
static uint8_t run_switch_raw, run_switch_stable;
static uint32_t run_switch_changed;
static void start_auto(uint8_t run2);
static void finish_retract1(void);
static void arc_test_update(void);
static void finish_run(void);
static void turn_at_outward_end(void);
/* Fixed pulse/position reference for each leg, also captured on resume. */
static uint32_t demo_leg_start_pulses, demo_leg_start_position;

/* Foam-clearance path, 24 Oct 2026. Anchors are servo degrees at DATA
 * positions measured from SW4 first touch (with overtravel disabled). The
 * final anchor is the counted outward reversal position, 5000 steps before the
 * measured SW1 hit. Monotone cubic Hermite interpolation: weighted harmonic
 * interior slopes, zero slope at the endpoints and on both sides of the
 * 100-degree plateau. Slopes are degrees per 1000 steps. The return uses this
 * exact same curve.
 */
static const struct {
    uint32_t step;
    float angle;
    float slope;
} demo_schedule[] = {
    {100000U, 90.0f, 0.0f},
    {105000U, 100.0f, 0.0f},
    {115000U, 100.0f, 0.0f},
    {125000U, 110.0f, 0.75f},
    {135000U, 116.0f, 0.6f},
    {155000U, 128.0f, 1.219093498f},
    {RUN_OUTWARD_END_STEPS, 135.0f, 0.0f}
};
#define DEMO_SCHEDULE_ROWS (sizeof demo_schedule / sizeof demo_schedule[0])

/* path_step()
 * -----------
 * Convert an internal controller position to steps from SW4 first touch.
 *
 * position: Counted coordinate including homing overtravel.
 *
 * Returns: Position minus overtravel, clamped at zero.
 * Global variables modified: None.
 * Errors: None.
 */
static uint32_t path_step(uint32_t position)
{
    return position > LIMIT_OVERTRAVEL_STEPS
            ? position - LIMIT_OVERTRAVEL_STEPS : 0U;
}

/* demo_pulse()
 * ------------
 * Interpolate the base Hermite path and round to whole microseconds. Clamp
 * outside the first/last anchors.
 *
 * step: Steps from SW4 first touch.
 *
 * Returns: Uncalibrated servo pulse in the 90-135 degree base range.
 * Global variables modified: None.
 * Errors: None.
 */
static unsigned demo_pulse(uint32_t step)
{
    if (step <= demo_schedule[0].step) {
        return DEMO_START_PULSE_US;
    }
    for (unsigned i = 1; i < DEMO_SCHEDULE_ROWS; ++i) {
        if (step <= demo_schedule[i].step) {
            float span = (float)(demo_schedule[i].step
                    - demo_schedule[i - 1].step);
            float t = (float)(step - demo_schedule[i - 1].step) / span;
            float t2 = t * t;
            float t3 = t2 * t;
            float angle = (2.0f * t3 - 3.0f * t2 + 1.0f)
                    * demo_schedule[i - 1].angle
                    + (t3 - 2.0f * t2 + t) * (span / 1000.0f)
                            * demo_schedule[i - 1].slope
                    + (-2.0f * t3 + 3.0f * t2) * demo_schedule[i].angle
                    + (t3 - t2) * (span / 1000.0f) * demo_schedule[i].slope;
            /* Bound round-off locally as well as protecting the overall arc. */
            if (angle < demo_schedule[i - 1].angle) {
                angle = demo_schedule[i - 1].angle;
            }
            if (angle > demo_schedule[i].angle) {
                angle = demo_schedule[i].angle;
            }
            unsigned pulse = (unsigned)(SERVO_MIN_PULSE_US
                    + angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)
                            / SERVO_MAX_ANGLE + 0.5f);
            if (pulse < DEMO_START_PULSE_US) {
                return DEMO_START_PULSE_US;
            }
            return pulse > DEMO_END_PULSE_US ? DEMO_END_PULSE_US : pulse;
        }
    }
    return DEMO_END_PULSE_US; /* Clamp at the final path angle. */
}

/* demo_servo()
 * ------------
 * Apply the saved straight offset and update servo 2 only if its target
 * changes. This helper does not enable PWM.
 *
 * pulse: Uncalibrated base/expanded-path pulse; caller ensures range.
 *
 * Returns: None.
 * Global variables modified: status servo target and TIM4 compare preload.
 * Errors: Driver failure enters Error_Handler().
 */
static void demo_servo(unsigned pulse)
{
    pulse = (unsigned)((int)pulse
            + (int)SERVO_ANGLE_US(status.straight_angle)
            - (int)DEMO_START_PULSE_US);
    if (status.servo_pulse_us[1] == (int)pulse) {
        return;
    }
    if (Servo_SetPulseUs(2U, pulse) != HAL_OK) {
        Error_Handler();
        return;
    }
    status.servo_pulse_us[1] = (int)pulse;
    status.servo_angle[1] = ((pulse - SERVO_MIN_PULSE_US)
            * SERVO_MAX_ANGLE + 1000U) / 2000U;
}

/* demo_update_position()
 * ----------------------
 * Recount travel from the fixed leg reference and command the matching
 * calibrated servo target. Homing leaves position unchanged.
 *
 * Returns: None.
 * Global variables modified: status.demo_position, return_steps and servo
 * target; TIM4 preload.
 * Errors: None.
 */
static void demo_update_position(void)
{
    uint32_t total = Stepper1_TotalPulses();
    uint32_t delta = total - demo_leg_start_pulses;
    if (status.demo_homing) {
        return;
    }
    if (status.target == 1U) {
        status.demo_position = delta > UINT32_MAX - demo_leg_start_position
                ? UINT32_MAX : demo_leg_start_position + delta;
    } else {
        /* Retrace from the ACTUAL turnaround, never from a hard-coded
         * endpoint. Keep a fixed reference for the whole return, including
         * switch bounce stops.
         */
        status.return_steps = delta;
        status.demo_position = delta > demo_leg_start_position
                ? 0U : demo_leg_start_position - delta;
    }
    uint32_t step = path_step(status.demo_position);
    /* Stretch cut 2's curve to begin at the intermediate retract endpoint.
     * Clamp before mapping so its final return holds straight below that
     * point.
     */
    if (status.run2 && status.run2_phase >= RUN2_CUT2) {
        if (step <= RUN2_CURVE_START_STEPS) {
            step = demo_schedule[0].step;
        } else if (step >= RUN_OUTWARD_END_STEPS) {
            step = RUN_OUTWARD_END_STEPS;
        } else {
            step = demo_schedule[0].step
                    + (uint32_t)((uint64_t)(step - RUN2_CURVE_START_STEPS)
                            * (RUN_OUTWARD_END_STEPS - demo_schedule[0].step)
                            / (RUN_OUTWARD_END_STEPS - RUN2_CURVE_START_STEPS));
        }
    }
    unsigned pulse = demo_pulse(step);
    /* Scale the same smooth curve about straight for cut 2 and its return. */
    if (status.run2 && status.run2_phase >= RUN2_CUT2) {
        unsigned span = DEMO_END_PULSE_US - DEMO_START_PULSE_US;
        pulse = DEMO_START_PULSE_US + ((pulse - DEMO_START_PULSE_US)
                * (SERVO_ANGLE_US(RUN2_END_ANGLE) - DEMO_START_PULSE_US)
                + span / 2U) / span;
    }
    demo_servo(pulse);
}

/* run_switch_changed_state()
 * --------------------------
 * Debounce both PC11 edges using unsigned tick subtraction.
 *
 * Returns: 1 once per accepted stable edge; otherwise 0.
 * Global variables modified: run_switch_raw, run_switch_stable and
 * run_switch_changed.
 * Errors: None.
 */
static uint8_t run_switch_changed_state(void)
{
    uint8_t pressed = Switch_IsPressed(2);
    uint32_t now = HAL_GetTick();
    if (pressed != run_switch_raw) {
        run_switch_raw = pressed;
        run_switch_changed = now;
    }
    if (run_switch_stable != run_switch_raw
            && (uint32_t)(now - run_switch_changed) >= SWITCH_DEBOUNCE_MS) {
        run_switch_stable = run_switch_raw;
        return 1; /* Both ON and OFF transitions must debounce. */
    }
    return 0;
}

/* target_direction()
 * ------------------
 * Translate the current target switch into a DIR pin level.
 *
 * Returns: Zero toward SW4, one toward SW1.
 * Global variables modified: None.
 * Errors: None.
 */
static uint8_t target_direction(void)
{
    return status.target == 4U ? DIR_TO_SWITCH4 : (DIR_TO_SWITCH4 ^ 1U);
}

/* run_speed()
 * -----------
 * Select the profile speed for the active phase. Homing is fixed; cut 2 has
 * no straight-speed segment.
 *
 * Returns: STEP pulses/s for the current phase.
 * Global variables modified: None.
 * Errors: None.
 */
static uint32_t run_speed(void)
{
    if (status.demo_homing) {
        return RUN_HOMING_SPEED;
    }
    if (status.run2) {
        if (status.run2_phase == RUN2_RETRACT1) {
            return status.run2_speeds[2];
        }
        if (status.run2_phase == RUN2_RETURN2) {
            return status.run2_speeds[4];
        }
        /* Cut 2 turns immediately, so it does not use a straight-speed slot. */
        if (status.run2_phase == RUN2_CUT2) {
            return status.run2_speeds[3];
        }
        return status.run2_speeds[path_step(status.demo_position)
                >= demo_schedule[0].step ? 1U : 0U];
    }
    if (status.target == 4U) {
        return status.return_speed;
    }
    return path_step(status.demo_position) >= demo_schedule[0].step
            ? status.curve_speed : status.straight_speed;
}

/* start_target()
 * --------------
 * Start the stopped stepper toward the current target. Count cut/retract
 * travel; use continuous pulses for homing/final return.
 *
 * Returns: None.
 * Global variables modified: status.speed, phase/turn state at endpoints,
 * and stepper registers.
 * Errors: Caller must stop STEP output before entry; driver failures halt.
 */
static void start_target(void)
{
    if (status.state == ROBOT_RUNNING) {
        status.speed = run_speed();
        Stepper1_SetSpeed(status.speed);
    }
    if (status.state == ROBOT_RUNNING && !status.demo_homing
            && status.target == 1U) {
        if (status.demo_position >= RUN_OUTWARD_STOP_POSITION) {
            turn_at_outward_end();
        } else {
            Stepper1_Move(target_direction(),
                    RUN_OUTWARD_STOP_POSITION - status.demo_position);
        }
    } else if (status.state == ROBOT_RUNNING && !status.demo_homing
            && status.run2 && status.run2_phase == RUN2_RETRACT1) {
        if (status.demo_position <= RUN2_RETRACT_POSITION) {
            finish_retract1();
        } else {
            Stepper1_Move(target_direction(),
                    status.demo_position - RUN2_RETRACT_POSITION);
        }
    } else {
        Stepper1_Start(target_direction());
    }
}

/* update_run_speed()
 * ------------------
 * Change phase speed at a path boundary without losing an in-flight pulse.
 * Ignore paused, homing, idle or limit-handling states.
 *
 * Returns: None.
 * Global variables modified: status position/speed/servo target and stepper
 * registers.
 * Errors: None.
 */
static void update_run_speed(void)
{
    if (status.state != ROBOT_RUNNING || status.paused || status.demo_homing
            || debouncing || overtravel || !Stepper1_IsBusy()
            || status.speed == run_speed()) {
        return;
    }
    Stepper1_Stop();
    demo_update_position();
    start_target();
}

/* turn_around()
 * -------------
 * Record a reversal, reset the leg reference and start the next direction.
 * Completing homing also establishes the counted origin.
 *
 * Returns: None.
 * Global variables modified: status, demo_leg_start_position,
 * demo_leg_start_pulses and drivers.
 * Errors: None.
 */
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
        status.duty = status.target == 1U
                ? DEMO_OUTWARD_DC_DUTY : DEMO_RETURN_DC_DUTY;
        Motor1_SetDuty(status.duty);
    }
    start_target();
}

/* turn_at_outward_end()
 * ---------------------
 * Advance the run2 cut phase if applicable, reverse and mark a counted turn.
 *
 * Returns: None.
 * Global variables modified: status phase/turn fields, leg references and
 * actuator outputs.
 * Errors: None.
 */
static void turn_at_outward_end(void)
{
    if (status.run2) {
        status.run2_phase = status.run2_phase == RUN2_CUT1
                ? RUN2_RETRACT1 : RUN2_RETURN2;
    }
    turn_around();
    status.turn_at_position = 1;
}

/* finish_retract1()
 * -----------------
 * Start cut 2 at the counted retract endpoint without rehoming or zeroing
 * absolute position.
 *
 * Returns: None.
 * Global variables modified: status phase/turn/duty fields, leg references
 * and driver outputs.
 * Errors: None.
 */
static void finish_retract1(void)
{
    status.turn_at_position = 1U;
    status.turn_switch = 4U;
    status.turn_position = status.demo_position;
    status.turn_pulse_us = (unsigned)status.servo_pulse_us[1];
    ++status.turn_number;
    status.run2_phase = RUN2_CUT2;
    status.target = 1U;
    status.return_steps = 0U;
    demo_leg_start_position = status.demo_position;
    demo_leg_start_pulses = Stepper1_TotalPulses();
    status.duty = DEMO_OUTWARD_DC_DUTY;
    Motor1_SetDuty(status.duty);
    start_target();
}

/* finish_run()
 * ------------
 * Complete at confirmed SW4 first touch, hold calibrated straight and stop
 * the drill/stepper.
 *
 * Returns: None.
 * Global variables modified: status, debounce/overtravel state and actuator
 * outputs.
 * Errors: None.
 */
static void finish_run(void)
{
    status.demo_position = RUN_HOME_POSITION;
    demo_servo(DEMO_START_PULSE_US);
    Robot_Stop();
    status.run_complete = 1U;
}

/* limit_reached()
 * ---------------
 * Finish an inward non-homing run at SW4; otherwise reverse after any
 * configured overtravel.
 *
 * Returns: None.
 * Global variables modified: Controller phase/turn state, leg references and
 * drivers.
 * Errors: None.
 */
static void limit_reached(void)
{
    if (status.state == ROBOT_RUNNING && status.target == 4U
            && !status.demo_homing) {
        finish_run();
    } else {
        turn_around();
    }
}

/* limit_confirmed()
 * -----------------
 * Handle a confirmed target: finish final return, fault on unexpected run2
 * limits, or drive bounded overtravel.
 *
 * Returns: None.
 * Global variables modified: status, overtravel, leg references and driver
 * outputs.
 * Errors: Unexpected run2 limit latches ROBOT_FAULT_RUN2_LIMIT and stops.
 */
static void limit_confirmed(void)
{
    uint32_t steps = LIMIT_OVERTRAVEL_STEPS;
    if (status.state == ROBOT_RUNNING && status.run2 && !status.demo_homing
            && status.run2_phase != RUN2_RETURN2) {
        status.fault = ROBOT_FAULT_RUN2_LIMIT;
        Robot_Stop();
        return;
    }
    if (status.state == ROBOT_RUNNING && status.target == 4U
            && !status.demo_homing) {
        int64_t error = (int64_t)status.outward_steps - status.return_steps
                - RUN_HOME_POSITION;
        status.home_step_error = error > INT32_MAX ? INT32_MAX :
                error < INT32_MIN ? INT32_MIN : (int32_t)error;
        finish_run();
        return;
    }
    if (status.state == ROBOT_RUNNING && !status.demo_homing
            && status.target == 1U) {
        uint32_t left = status.demo_position < RUN_OUTWARD_STOP_POSITION ?
                RUN_OUTWARD_STOP_POSITION - status.demo_position : 0U;
        /* Never overtravel beyond the counted outward end. */
        if (steps > left) {
            steps = left;
        }
    }
    if (steps) {
        overtravel = 1;
        Stepper1_Move(target_direction(), steps);
    } else {
        limit_reached();
    }
}

/* Robot_Init()
 * ------------
 * Initialise the controller and drivers after CubeMX setup. Motors start
 * stopped and servo PWM starts disabled.
 *
 * motor: Initialised TIM1 drill handle.
 * stepper: Initialised TIM16 STEP handle.
 * servos: Initialised TIM4 servo handle.
 *
 * Returns: None.
 * Global variables modified: status, debounce/run-switch state, and actuator
 * driver state.
 * Errors: Invalid driver handles or failed PWM setup enter Error_Handler().
 */
void Robot_Init(TIM_HandleTypeDef* motor, TIM_HandleTypeDef* stepper,
        TIM_HandleTypeDef* servos)
{
    status.state = ROBOT_IDLE;
    status.step_mode = ROBOT_STEP_STOPPED;
    status.straight_speed = 750U;
    status.curve_speed = 500U;
    status.return_speed = 2000U;
    /* straight1, curve1, retract1, curve2, return2 (STEP pulses/s). */
    const uint32_t defaults[] = {750U, 500U, 2000U, 800U, 2000U};
    for (unsigned i = 0; i < 5; ++i) {
        status.run2_speeds[i] = defaults[i];
    }
    status.run2 = 0;
    status.run2_phase = RUN2_CUT1;
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

/* Robot_Stop()
 * ------------
 * Cancel autonomous/manual movement and the arc test. Stop STEP pulses and
 * drill duty while retaining servo PWM, calibration and faults.
 *
 * Returns: None.
 * Global variables modified: status, debouncing, overtravel and
 * stepper/motor state.
 * Errors: None.
 */
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

/* Robot_Update()
 * --------------
 * Service limits, the maintained run switch, run phases and servo position
 * without waiting for travel. Call from every main-loop pass.
 *
 * Returns: None.
 * Global variables modified: status, debounce/overtravel state, run-switch
 * state and driver outputs.
 * Errors: Limit faults latch in status.fault and stop motion.
 */
void Robot_Update(void)
{
    uint8_t switch_changed = run_switch_changed_state();
    if (!status.fault && Switch_IsPressed(1) && Switch_IsPressed(4)) {
        status.fault = ROBOT_FAULT_BOTH_LIMITS;
    }
    if (status.fault) {
        Robot_Stop();
        return;
    }
    if (switch_changed) {
        if (!run_switch_stable) {
            Robot_Stop();
        } else if (status.state != ROBOT_RUNNING) {
            /* The switch uses the same calibrated range restriction as CLI
             * run2. An invalid range must never start movement; status
             * explains the block.
             */
            if (status.straight_angle <= SERVO_MAX_ANGLE
                    - (RUN2_END_ANGLE - DEMO_START_ANGLE)) {
                start_auto(1);
            }
        }
    }
    if (status.state == ROBOT_RUNNING) {
        demo_update_position();
    }
    if (status.state == ROBOT_RUNNING && status.paused) {
        return; /* Frozen until continue or stop. */
    }
    if (status.arc_test) {
        arc_test_update();
    }
    if (status.step_mode == ROBOT_STEP_MANUAL) {
        if (Switch_IsPressed(status.target) || !Stepper1_IsBusy()) {
            Stepper1_Stop();
            status.step_mode = ROBOT_STEP_STOPPED;
        }
    } else if (status.step_mode == ROBOT_STEP_AUTO) {
        if (status.demo_backoff) {
            /* The run started on SW4. When the counted back-off ends, SW4 must
               be released before homing. The debounce window starts
               when the move ends. */
            if (Stepper1_IsBusy()) {
                debounce_start = HAL_GetTick();
            } else if (!Switch_IsPressed(4)) {
                status.demo_backoff = 0;
                start_target();
            } else if ((uint32_t)(HAL_GetTick() - debounce_start)
                    >= SWITCH_DEBOUNCE_MS) {
                /* Latch a fault if SW4 remains pressed after backing off. */
                status.fault = ROBOT_FAULT_SW4_STUCK;
                Robot_Stop();
            }
        } else if (overtravel) {
            if (!Stepper1_IsBusy()) {
                overtravel = 0;
                if (status.state == ROBOT_RUNNING) {
                    demo_update_position();
                }
                limit_reached();
            }
        } else if (debouncing) {
            if ((uint32_t)(HAL_GetTick() - debounce_start)
                    >= SWITCH_DEBOUNCE_MS) {
                debouncing = 0;
                if (Switch_IsPressed(status.target)) {
                    limit_confirmed();
                } else {
                    /* A false press resumes toward the same target. */
                    start_target();
                }
            }
        } else if (Switch_IsPressed(status.target)) {
            Stepper1_Stop();
            if (status.state == ROBOT_RUNNING) {
                demo_update_position();
            }
            debouncing = 1;
            debounce_start = HAL_GetTick();
        }
    }
    if (status.state == ROBOT_RUNNING && !status.demo_homing
            && status.target == 1U && !debouncing && !overtravel
            && !Stepper1_IsBusy()) {
        turn_at_outward_end();
    }
    if (status.state == ROBOT_RUNNING && !status.demo_homing && status.run2
            && status.run2_phase == RUN2_RETRACT1 && !debouncing && !overtravel
            && !Stepper1_IsBusy()) {
        finish_retract1();
    }
    update_run_speed();
}

/* arc_test_update()
 * -----------------
 * Advance the timed out/hold/back servo test and finish at calibrated
 * straight.
 *
 * Returns: None.
 * Global variables modified: status.arc_test, arc_tests_done and servo
 * target; TIM4 preload.
 * Errors: None.
 */
static void arc_test_update(void)
{
    uint32_t t = HAL_GetTick() - arc_test_start;
    uint32_t span = DEMO_END_PULSE_US - DEMO_START_PULSE_US;
    unsigned pulse = DEMO_START_PULSE_US;
    if (t < ARC_TEST_SWEEP_MS) {
        pulse = DEMO_START_PULSE_US
                + (t * span + ARC_TEST_SWEEP_MS / 2U) / ARC_TEST_SWEEP_MS;
    } else if (t < ARC_TEST_SWEEP_MS + ARC_TEST_HOLD_MS) {
        pulse = DEMO_END_PULSE_US;
    } else if (t < 2U * ARC_TEST_SWEEP_MS + ARC_TEST_HOLD_MS) {
        pulse = DEMO_END_PULSE_US
                - ((t - ARC_TEST_SWEEP_MS - ARC_TEST_HOLD_MS) * span
                        + ARC_TEST_SWEEP_MS / 2U) / ARC_TEST_SWEEP_MS;
    } else {
        status.arc_test = 0;
        ++status.arc_tests_done;
    }
    demo_servo(pulse);
}

/* motion_allowed()
 * ----------------
 * Refresh controller state and gate a CLI motion request.
 *
 * individual: Nonzero for actuator commands blocked during a run.
 *
 * Returns: NULL if permitted; a static error string otherwise.
 * Global variables modified: Controller/driver state through Robot_Update().
 * Errors: Reports a latched fault or, for individual commands, an active
 * run.
 */
static const char* motion_allowed(uint8_t individual)
{
    Robot_Update();
    if (status.fault) {
        return "LIMIT FAULT LATCHED: RESET REQUIRED";
    }
    if (individual && status.state == ROBOT_RUNNING) {
        return "AUTONOMOUS ACTIVE: send stop first";
    }
    return NULL;
}

/* begin_auto()
 * ------------
 * Start the stepper-only test toward the end switch not already pressed.
 *
 * Returns: None.
 * Global variables modified: status target/mode/backoff, debounce/overtravel
 * and stepper state.
 * Errors: None.
 */
static void begin_auto(void)
{
    Stepper1_Stop();
    debouncing = overtravel = 0;
    status.demo_backoff = 0;
    status.target = Switch_IsPressed(4) ? 1U : 4U;
    status.step_mode = ROBOT_STEP_AUTO;
    start_target();
}

/* start_auto()
 * ------------
 * Reset bookkeeping and start fixed-speed homing, backing off SW4 first when
 * pressed. Callers validate faults and calibration.
 *
 * run2: Nonzero selects two cuts; zero selects one cut.
 *
 * Returns: None.
 * Global variables modified: status, leg references, debounce/overtravel and
 * actuator drivers.
 * Errors: Servo driver failure enters Error_Handler().
 */
static void start_auto(uint8_t run2)
{
    Stepper1_Stop();
    status.run2 = run2;
    status.run2_phase = RUN2_CUT1;
    status.speed = RUN_HOMING_SPEED;
    Stepper1_SetSpeed(status.speed);
    Motor1_SetDuty(0);
    /* Re-home on every run; never treat an arbitrary starting point as zero. A
     * running servo PWM takes the new width at its next period, without a
     * glitch.
     */
    status.servo_pulse_us[1] = -1;
    demo_servo(DEMO_START_PULSE_US);
    if (Servo_Start(2U) != HAL_OK) {
        Error_Handler();
        return;
    }
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
        /* Move off a held SW4 before homing and setting position zero. */
        status.demo_backoff = 1U;
        Stepper1_Move(DIR_TO_SWITCH4 ^ 1U, RUN_START_BACKOFF_STEPS);
    } else {
        start_target();
    }
}

/* Robot_StartAuto()
 * -----------------
 * Start one calibrated 90-135 degree cut, then return to physical SW4. An
 * already active single-cut run is left running.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: Controller run state and actuator outputs.
 * Errors: Rejects faults, paused runs or an active run2.
 */
const char* Robot_StartAuto(void)
{
    const char* error = motion_allowed(0);
    if (error) {
        return error;
    }
    if (status.paused) {
        return "RUN PAUSED: send continue or stop";
    }
    if (status.state == ROBOT_RUNNING && status.run2) {
        return "RUN2 ACTIVE: send stop first";
    }
    if (status.state != ROBOT_RUNNING) {
        start_auto(0);
    }
    return NULL;
}

/* Robot_StartAuto2()
 * ------------------
 * Start two cuts (135 then 145 degrees), retracting to 95000 between cuts,
 * then return to physical SW4. Angles are before calibration.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: Controller run state and actuator outputs.
 * Errors: Rejects faults, pause, an active run, or straight calibration
 * above 125.
 */
const char* Robot_StartAuto2(void)
{
    const char* error = motion_allowed(0);
    if (error) {
        return error;
    }
    if (status.paused) {
        return "RUN PAUSED: send continue or stop";
    }
    if (status.state == ROBOT_RUNNING && !status.run2) {
        return "RUN ACTIVE: send stop first";
    }
    if (status.straight_angle > SERVO_MAX_ANGLE
            - (RUN2_END_ANGLE - DEMO_START_ANGLE)) {
        return "RUN2 REQUIRES CALIBRATION 0..125 "
                "(55-DEGREE SWEEP MUST FIT 0..180)";
    }
    if (status.state != ROBOT_RUNNING) {
        start_auto(1);
    }
    return NULL;
}

/* Robot_Pause()
 * -------------
 * Stop the stepper and drill but retain the active run phase and remaining
 * pulses. Servo holds; repeated pause is harmless.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: status.pause_remaining, status.paused, position
 * and driver state.
 * Errors: Rejects faults or absence of an active run.
 */
const char* Robot_Pause(void)
{
    const char* error = motion_allowed(0);
    if (error) {
        return error;
    }
    if (status.state != ROBOT_RUNNING) {
        return "NO RUN TO PAUSE";
    }
    if (status.paused) {
        return NULL;
    }
    status.pause_remaining = Stepper1_StopRemaining();
    demo_update_position();
    Motor1_SetDuty(0);
    status.duty = 0;
    status.paused = 1U;
    return NULL;
}

/* Robot_Continue()
 * ----------------
 * Resume the paused phase, counted move and drill direction. Restart a
 * pending switch debounce interval.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: status, debounce_start, leg references and
 * actuator outputs.
 * Errors: Rejects faults or absence of a paused run.
 */
const char* Robot_Continue(void)
{
    const char* error = motion_allowed(0);
    if (error) {
        return error;
    }
    if (status.state != ROBOT_RUNNING || !status.paused) {
        return "NOT PAUSED";
    }
    status.paused = 0;
    status.duty = status.demo_homing ? 0
            : status.target == 1U ? DEMO_OUTWARD_DC_DUTY : DEMO_RETURN_DC_DUTY;
    Motor1_SetDuty(status.duty);
    /* A switch being debounced must hold for a fresh 30 ms before it counts. */
    debounce_start = HAL_GetTick();
    if (debouncing) {
        return NULL;
    }
    if (status.demo_backoff || overtravel) {
        if (status.pause_remaining) {
            Stepper1_Move(status.demo_backoff
                            ? (DIR_TO_SWITCH4 ^ 1U) : target_direction(),
                    status.pause_remaining);
        }
    } else {
        start_target();
    }
    status.pause_remaining = 0;
    return NULL;
}

/* Robot_ServoArcTest()
 * --------------------
 * Start the calibrated timed servo-2 sweep. Does not start or stop
 * independently commanded motor motion.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: status, arc_test_start and servo PWM/target.
 * Errors: Rejects faults, active runs or servo driver failure.
 */
const char* Robot_ServoArcTest(void)
{
    const char* error = motion_allowed(1);
    if (error) {
        return error;
    }
    demo_servo(DEMO_START_PULSE_US);
    if (Servo_Start(2U) != HAL_OK) {
        return "SERVO DRIVER ERROR";
    }
    arc_test_start = HAL_GetTick();
    status.arc_test = 1U;
    status.state = ROBOT_IDLE;
    return NULL;
}

/* Robot_StepperAuto()
 * -------------------
 * Start the stepper-only limit-to-limit test, replacing a manual counted
 * move. Leave drill duty unchanged.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: status, debouncing, overtravel and stepper
 * state.
 * Errors: Rejects faults or an active autonomous run.
 */
const char* Robot_StepperAuto(void)
{
    const char* error = motion_allowed(1);
    if (error) {
        return error;
    }
    begin_auto();
    status.state = ROBOT_IDLE;
    return NULL;
}

/* Robot_Move()
 * ------------
 * Start a counted manual move; target-limit detection can stop it early.
 *
 * steps: Signed pulse count, magnitude 1..1000000. Positive moves
 *     toward SW4; negative moves toward SW1.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: status target/mode/state and stepper driver.
 * Errors: Rejects invalid count, faults, active run, busy stepper or pressed
 * target.
 */
const char* Robot_Move(int32_t steps)
{
    if (!steps || steps < -1000000 || steps > 1000000) {
        return "STEPS MUST BE -1000000..-1 OR 1..1000000";
    }
    const char* error = motion_allowed(1);
    if (error) {
        return error;
    }
    uint8_t target = steps > 0 ? 4U : 1U;
    if (Switch_IsPressed(target)) {
        return "TARGET LIMIT ACTIVE";
    }
    if (status.step_mode != ROBOT_STEP_STOPPED) {
        return "STEPPER BUSY: send stepper(stop) first";
    }
    status.target = target;
    status.step_mode = ROBOT_STEP_MANUAL;
    status.state = ROBOT_IDLE;
    Stepper1_Move(target == 4U ? DIR_TO_SWITCH4 : (DIR_TO_SWITCH4 ^ 1U),
                  (uint32_t)(steps < 0 ? -steps : steps));
    return NULL;
}

/* Robot_SetRunSpeeds()
 * --------------------
 * Validate and set the run profile atomically; select straight as the manual
 * speed. Homing stays at 2000 steps/s.
 *
 * straight: Straight-section speed, 16..2000 STEP pulses/s.
 * curve: Turning-section speed, 16..2000 STEP pulses/s.
 * reverse: Return speed, 16..2000 STEP pulses/s.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: status run profile/speed and stepper timer
 * period.
 * Errors: Rejects invalid speeds or active stepper/run state.
 */
const char* Robot_SetRunSpeeds(
        uint32_t straight, uint32_t curve, uint32_t reverse)
{
    if (straight < 16 || straight > 2000 || curve < 16 || curve > 2000
            || reverse < 16 || reverse > 2000) {
        return "INVALID SPEED: USE 16..2000 STEPS/S";
    }
    if (status.state == ROBOT_RUNNING
            || status.step_mode != ROBOT_STEP_STOPPED) {
        return "STOP STEPPER BEFORE CHANGING SPEED";
    }
    status.straight_speed = straight;
    status.curve_speed = curve;
    status.return_speed = reverse;
    status.speed = straight;
    Stepper1_SetSpeed(straight);
    return NULL;
}

/* Robot_SetRun2Speeds()
 * ---------------------
 * Validate and copy the five run2 speeds atomically. Does not start movement
 * or change the fixed homing speed.
 *
 * speeds: Five readable values: straight1, curve1, retract1, curve2,
 *     return2; each is 16..2000 STEP pulses/s.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: status.run2_speeds.
 * Errors: Rejects invalid speeds or active stepper/run state.
 */
const char* Robot_SetRun2Speeds(const uint32_t speeds[5])
{
    for (unsigned i = 0; i < 5; ++i) {
        if (speeds[i] < 16U || speeds[i] > 2000U) {
            return "INVALID SPEED: USE 16..2000 STEPS/S";
        }
    }
    if (status.state == ROBOT_RUNNING
            || status.step_mode != ROBOT_STEP_STOPPED) {
        return "STOP STEPPER BEFORE CHANGING SPEED";
    }
    for (unsigned i = 0; i < 5; ++i) {
        status.run2_speeds[i] = speeds[i];
    }
    return NULL;
}

/* Robot_SetSpeed()
 * ----------------
 * Set all three run speeds and the manual speed together; leave run2 and
 * homing speeds unchanged.
 *
 * speed: 16..2000 STEP pulses/s.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: status run profile/speed and stepper timer
 * period.
 * Errors: Rejects invalid speed or active stepper/run state.
 */
const char* Robot_SetSpeed(uint32_t speed)
{
    return Robot_SetRunSpeeds(speed, speed, speed);
}

/* Robot_SetMotor()
 * ----------------
 * Set signed drill duty. Zero also cancels an active autonomous run.
 *
 * duty: -100..100 percent; sign selects bridge direction.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: status duty/state and motor PWM; zero may stop
 * the full run.
 * Errors: Rejects invalid duty; nonzero duty also rejects faults or active
 * runs.
 */
const char* Robot_SetMotor(int duty)
{
    if (duty < -100 || duty > 100) {
        return "DUTY OUT OF RANGE: USE -100..100";
    }
    if (!duty) {
        Robot_StopMotor();
        return NULL;
    }
    const char* error = motion_allowed(1);
    if (error) {
        return error;
    }
    Motor1_SetDuty((int)duty);
    status.duty = duty;
    status.state = ROBOT_IDLE;
    return NULL;
}

/* Robot_Calibrate()
 * -----------------
 * Save a session-only straight angle and command servo 2 there. Reset
 * restores 90 degrees; no calibration is saved if the command fails.
 *
 * angle: Absolute straight angle, 0..135 degrees. Starting run2
 *     additionally requires an angle no greater than 125.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: status.straight_angle, servo target/state and
 * servo PWM.
 * Errors: Rejects invalid angle, faults, active motion or servo driver
 * failure.
 */
const char* Robot_Calibrate(unsigned angle)
{
    if (angle > SERVO_MAX_ANGLE - (DEMO_END_ANGLE - DEMO_START_ANGLE)) {
        return "CALIBRATION OUT OF RANGE: USE 0..135 "
                "(FULL SWEEP MUST FIT 0..180)";
    }
    const char* error = motion_allowed(1);
    if (error) {
        return error;
    }
    if (status.step_mode != ROBOT_STEP_STOPPED
            || status.duty || status.arc_test) {
        return "STOP ALL MOTION BEFORE CALIBRATING";
    }
    /* Keep manual angles absolute so the user can first find straight. */
    error = Robot_SetServo(2U, angle);
    if (error) {
        return error;
    }
    status.straight_angle = angle;
    return NULL;
}

/* Robot_SetServo()
 * ----------------
 * Set and enable an absolute servo angle without adding calibration. Cancel
 * any servo arc test.
 *
 * id: Servo 1 or 2; the CLI exposes only servo 2.
 * angle: Absolute nominal angle, 0..180 degrees.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: status servo target, arc_test/state and servo
 * PWM.
 * Errors: Rejects invalid id/angle, faults, active runs or driver failure.
 */
const char* Robot_SetServo(unsigned id, unsigned angle)
{
    if (id < 1 || id > 2) {
        return "INVALID SERVO: USE 1 OR 2";
    }
    if (angle > SERVO_MAX_ANGLE) {
        return "ANGLE OUT OF RANGE: USE 0..180";
    }
    const char* error = motion_allowed(1);
    if (error) {
        return error;
    }
    if (servo((uint8_t)id, (int)angle) != HAL_OK
            || Servo_Start((uint8_t)id) != HAL_OK) {
        return "SERVO DRIVER ERROR";
    }
    status.arc_test = 0; /* A manual target ends any sweep. */
    status.servo_angle[id - 1] = (int)angle;
    status.servo_pulse_us[id - 1] = (int)SERVO_ANGLE_US(angle);
    status.state = ROBOT_IDLE;
    return NULL;
}

/* Robot_SetServoPulseUs()
 * -----------------------
 * Set and enable an absolute servo pulse width without adding calibration.
 * Cancel any servo arc test.
 *
 * id: Servo 1 or 2.
 * pulse_us: Absolute pulse width, 500..2500 microseconds.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: status servo target, arc_test/state and servo
 * PWM.
 * Errors: Rejects invalid id/pulse, faults, active runs or driver failure.
 */
const char* Robot_SetServoPulseUs(unsigned id, unsigned pulse_us)
{
    if (id < 1 || id > 2) {
        return "INVALID SERVO: USE 1 OR 2";
    }
    if (pulse_us < SERVO_MIN_PULSE_US || pulse_us > SERVO_MAX_PULSE_US) {
        return "PULSE OUT OF RANGE: USE 500..2500 US";
    }
    const char* error = motion_allowed(1);
    if (error) {
        return error;
    }
    if (Servo_SetPulseUs((uint8_t)id, pulse_us) != HAL_OK
            || Servo_Start((uint8_t)id) != HAL_OK) {
        return "SERVO DRIVER ERROR";
    }
    status.arc_test = 0; /* A manual target ends any sweep. */
    status.servo_pulse_us[id - 1] = (int)pulse_us;
    status.servo_angle[id - 1] = ((pulse_us - SERVO_MIN_PULSE_US)
            * SERVO_MAX_ANGLE + (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 2)
            / (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    status.state = ROBOT_IDLE;
    return NULL;
}

/* Robot_StopStepper()
 * -------------------
 * Stop the stepper; during a run, cancel the entire run. Outside a run leave
 * the drill and servo unchanged.
 *
 * Returns: None.
 * Global variables modified: status, stepper state, debounce/overtravel; run
 * stop also clears duty.
 * Errors: None.
 */
void Robot_StopStepper(void)
{
    if (status.state == ROBOT_RUNNING) {
        Robot_Stop();
        return;
    }
    Stepper1_Stop();
    status.step_mode = ROBOT_STEP_STOPPED;
    debouncing = overtravel = 0;
}

/* Robot_StopMotor()
 * -----------------
 * Stop the drill; during a run, cancel the entire run. Outside a run retain
 * manual stepper motion and servo targets.
 *
 * Returns: None.
 * Global variables modified: status.duty and motor PWM; an active run
 * invokes Robot_Stop().
 * Errors: None.
 */
void Robot_StopMotor(void)
{
    if (status.state == ROBOT_RUNNING) {
        Robot_Stop();
        return;
    }
    Motor1_SetDuty(0);
    status.duty = 0;
}

/* Robot_GetStatus()
 * -----------------
 * Expose the controller-owned live status to main-loop readers.
 *
 * Returns: Read-only pointer valid for the lifetime of the firmware.
 * Global variables modified: None.
 * Errors: None.
 */
const RobotStatus* Robot_GetStatus(void)
{
    return &status;
}
