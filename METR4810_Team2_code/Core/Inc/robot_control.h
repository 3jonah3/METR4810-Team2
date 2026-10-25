#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "stm32g4xx_hal.h"
#include "servo.h"

/* Optional travel beyond a debounced homing/limit press, in STEP pulses.
 * Leave disabled until dampened switch mounts are fitted. Final return to
 * physical SW4 never uses overtravel. The internal origin is behind SW4 when
 * this option is enabled; path coordinates compensate for that shift.
 */
#ifndef LIMIT_OVERTRAVEL_STEPS
#define LIMIT_OVERTRAVEL_STEPS 0U
#endif

/* Reverse 5000 steps before the measured SW1 hit at 162297. */
#define RUN_OUTWARD_END_STEPS 157297U
#define RUN_OUTWARD_STOP_POSITION \
    (RUN_OUTWARD_END_STEPS + LIMIT_OVERTRAVEL_STEPS)

/* Both programs home/back off at this fixed speed, in STEP pulses/s. */
#define RUN_HOMING_SPEED 2000U
#define RUN2_CURVE_START_STEPS 95000U
#define RUN2_RETRACT_POSITION \
    (RUN2_CURVE_START_STEPS + LIMIT_OVERTRAVEL_STEPS)
#define RUN2_END_ANGLE 145

/* Unexpected physical limit during a counted run2 leg. */
#define ROBOT_FAULT_RUN2_LIMIT 3U

/* Run2 phases after homing, in execution order. */
typedef enum {
    RUN2_CUT1,
    RUN2_RETRACT1,
    RUN2_CUT2,
    RUN2_RETURN2
} RobotRun2Phase;

/* Expected SW4 first-touch position in the internal coordinate system. */
#define RUN_HOME_POSITION LIMIT_OVERTRAVEL_STEPS
#define RUN_START_BACKOFF_STEPS (1500U + LIMIT_OVERTRAVEL_STEPS)

/* Nonzero faults stop movement and remain latched until reset. */
#define ROBOT_FAULT_BOTH_LIMITS 1U /* SW1 and SW4 pressed together. */
#define ROBOT_FAULT_SW4_STUCK 2U /* SW4 still pressed after backing off. */

/* Base arc for run, cut 1 and servo(arc); cut 2 expands this to 145 degrees. */
#define DEMO_START_ANGLE 90
#define DEMO_START_PULSE_US SERVO_ANGLE_US(DEMO_START_ANGLE)
#define DEMO_END_ANGLE 135
#define DEMO_END_PULSE_US SERVO_ANGLE_US(DEMO_END_ANGLE)

/* Timed servo test and nominal pulse-to-pod-angle scale through the gears. */
#define ARC_TEST_SWEEP_MS 4000U
#define ARC_TEST_HOLD_MS 1000U
#define DEMO_US_PER_POD_DEG 20U

/* RUNNING includes paused runs; IDLE permits independent actuator commands. */
typedef enum {
    ROBOT_IDLE,
    ROBOT_RUNNING,
    ROBOT_STOPPED
} RobotState;

/* AUTO includes full programs and the stepper-only limit-to-limit test. */
typedef enum {
    ROBOT_STEP_STOPPED,
    ROBOT_STEP_AUTO,
    ROBOT_STEP_MANUAL
} RobotStepMode;

/* Live controller state, read-only outside robot_control.c. Position and
 * servo fields describe commands/counts, not measured mechanical feedback.
 */
typedef struct {
    RobotState state;
    RobotStepMode step_mode;

    /* Single-cut profile in STEP pulses/s. */
    uint32_t straight_speed;
    uint32_t curve_speed;
    uint32_t return_speed;
    /* straight1, curve1, retract1, curve2, return2. */
    uint32_t run2_speeds[5];
    uint8_t run2;
    RobotRun2Phase run2_phase;
    uint32_t speed; /* Active stepper speed, STEP pulses/s. */
    uint8_t target; /* Direction target: SW1 or SW4. */
    uint8_t fault; /* Zero or a latched ROBOT_FAULT_* value. */
    int duty; /* Signed drill duty, -100..100 percent. */

    /* Absolute servo command corresponding to path angle 90 degrees. */
    unsigned straight_angle;
    int servo_angle[2]; /* Nominal degrees; -1 until commanded. */
    int servo_pulse_us[2]; /* Exact pulse widths; -1 until commanded. */

    uint32_t demo_position; /* STEP pulses from the homed origin. */
    uint8_t demo_homing; /* Includes optional homing overtravel. */
    uint8_t demo_backoff; /* Clearing an already-pressed SW4 before homing. */
    uint32_t outward_steps; /* Most recent outward endpoint. */
    uint32_t return_steps; /* Pulses emitted on the current return. */
    /* outward_steps - return_steps - homing overtravel at SW4 first touch. */
    int32_t home_step_error;

    uint32_t turn_number; /* Event counter used by serial reporting. */
    uint32_t turn_position;
    unsigned turn_pulse_us;
    uint8_t turn_at_position; /* Counted reversal rather than a limit hit. */
    uint8_t turn_switch; /* Direction target of the last turn. */
    uint8_t run_complete; /* Final return reached physical SW4. */
    uint8_t paused;
    uint32_t pause_remaining; /* Unissued pulses saved for continue. */
    uint8_t arc_test;
    uint32_t arc_tests_done; /* Tests completed at calibrated straight. */
} RobotStatus;

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
        TIM_HandleTypeDef* servos);

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
void Robot_Update(void);

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
void Robot_Stop(void);

/* Robot_GetStatus()
 * -----------------
 * Expose the controller-owned live status to main-loop readers.
 *
 * Returns: Read-only pointer valid for the lifetime of the firmware.
 * Global variables modified: None.
 * Errors: None.
 */
const RobotStatus* Robot_GetStatus(void);

/* CLI command functions return NULL on success or static error text. */

/* Robot_StartAuto()
 * -----------------
 * Start one calibrated 90-135 degree cut, then return to physical SW4. An
 * already active single-cut run is left running.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: Controller run state and actuator outputs.
 * Errors: Rejects faults, paused runs or an active run2.
 */
const char* Robot_StartAuto(void);

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
const char* Robot_StartAuto2(void);

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
const char* Robot_SetRun2Speeds(const uint32_t speeds[5]);

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
const char* Robot_Pause(void);

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
const char* Robot_Continue(void);

/* Robot_ServoArcTest()
 * --------------------
 * Start the calibrated timed servo-2 sweep. Does not start or stop
 * independently commanded motor motion.
 *
 * Returns: NULL on success; static error text otherwise.
 * Global variables modified: status, arc_test_start and servo PWM/target.
 * Errors: Rejects faults, active runs or servo driver failure.
 */
const char* Robot_ServoArcTest(void);

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
const char* Robot_StepperAuto(void);

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
const char* Robot_Move(int32_t steps);

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
const char* Robot_SetSpeed(uint32_t speed);

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
        uint32_t straight, uint32_t curve, uint32_t reverse);

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
const char* Robot_SetMotor(int duty);

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
const char* Robot_SetServo(unsigned id, unsigned angle);

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
const char* Robot_Calibrate(unsigned angle);

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
const char* Robot_SetServoPulseUs(unsigned id, unsigned pulse_us);

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
void Robot_StopStepper(void);

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
void Robot_StopMotor(void);
#endif
