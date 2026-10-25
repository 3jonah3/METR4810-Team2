/**
 * @file serial.c
 * Non-blocking UART command parser, telemetry and response queues. RX and TX
 * callbacks only publish buffer/event state; commands execute from main.
 * Each reply is queued atomically or dropped whole when there is no space.
 */

#include "serial.h"
#include "robot_control.h"
#include "stepper.h"
#include "switch.h"
#include "status_led.h"
#include <ctype.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RX_SIZE 256U
#define TX_SIZE 8192U
#define LINE_SIZE 96U
static UART_HandleTypeDef* port;
static uint8_t rx_byte, tx[TX_SIZE];
/* Both data and indices are volatile: publish each byte before the ISR head. */
static volatile uint8_t rx[RX_SIZE];
static volatile unsigned rx_head, rx_tail, rx_fault;
static volatile unsigned tx_tail, tx_active;
static unsigned tx_head, tx_dropped;
static char line[LINE_SIZE];
static unsigned line_length, discard_line;
static uint32_t reported_turn;
/* Run feedback: CSV DATA rows while running, plus event lines when a run
 * starts, stops or faults (switch and fault stops are otherwise silent).
 */
static uint8_t telemetry_on, was_running, was_fault, was_paused, was_arc;
static uint32_t arc_done_seen;
static uint32_t telemetry_ms, telemetry_last;
static const char* last_leg;
static const char data_header[] =
        "DATA,ms,leg,steps,mm,servo_us,pod_deg,dc,sw1,sw4\r\n";
static void help(void);
static const char* leg(const RobotStatus* s);

/* reply()
 * -------
 * Format and enqueue one complete terminal response from main context.
 *
 * format: printf-style format string.
 * ...: Arguments matching the format conversions.
 *
 * Returns: None.
 * Global variables modified: tx, tx_head and tx_dropped.
 * Errors: Oversized or queue-full messages are dropped whole and counted.
 */
static void reply(const char* format, ...)
{
    char message[640];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(message, sizeof message, format, args);
    va_end(args);
    if (length < 0 || (unsigned)length >= sizeof message) {
        ++tx_dropped;
        return;
    }
    unsigned free_bytes = (tx_tail + TX_SIZE - tx_head - 1U) % TX_SIZE;
    if ((unsigned)length > free_bytes) {
        ++tx_dropped;
        return;
    }
    for (int i = 0; i < length; ++i) {
        tx[tx_head] = (uint8_t)message[i];
        tx_head = (tx_head + 1U) % TX_SIZE;
    }
}

/* transmit()
 * ----------
 * Start the next contiguous queued TX chunk if the UART is idle.
 *
 * Returns: None.
 * Global variables modified: tx_active and UART transmit state.
 * Errors: A failed start clears tx_active so a later update can retry.
 */
static void transmit(void)
{
    if (!tx_active && tx_head != tx_tail) {
        unsigned count = tx_head > tx_tail
                ? tx_head - tx_tail : TX_SIZE - tx_tail;
        tx_active = count;
        if (HAL_UART_Transmit_IT(port, &tx[tx_tail], (uint16_t)count)
                != HAL_OK) {
            tx_active = 0;
        }
    }
}

/* Serial_Init()
 * -------------
 * Initialise queues, telemetry and interrupt reception, then queue
 * welcome/help text.
 *
 * uart: Initialised UART1 handle that remains valid for the session.
 *
 * Returns: None.
 * Global variables modified: port, RX/TX queues, command buffer and serial
 * event-tracking state.
 * Errors: Receive-start failure sets rx_fault for Serial_Update().
 */
void Serial_Init(UART_HandleTypeDef* uart)
{
    port = uart;
    rx_head = rx_tail = rx_fault = 0;
    tx_head = tx_tail = tx_active = tx_dropped = 0;
    line_length = discard_line = 0;
    reported_turn = Robot_GetStatus()->turn_number;
    telemetry_on = 1;
    telemetry_ms = 1000U;
    was_running = was_paused = was_arc = 0;
    arc_done_seen = Robot_GetStatus()->arc_tests_done;
    /* Report faults latched during Robot_Init on the first update. */
    was_fault = 0;
    last_leg = "-";
    if (HAL_UART_Receive_IT(port, &rx_byte, 1) != HAL_OK) {
        rx_fault = 1;
    }
    reply("\r\nWelcome to METR4810 Team 2 CLI\r\n\r\n ,_,\r\n(O,O)\r\n(   "
            ")\r\n-\"-\"------\r\n\r\nREADY: IDLE; type help\r\n\r\n");
    help();
}

/* HAL_UART_RxCpltCallback()
 * -------------------------
 * Publish a received byte and rearm RX for the bound UART; ignore other
 * UARTs. Runs in interrupt context.
 *
 * uart: HAL handle whose receive operation completed.
 *
 * Returns: None.
 * Global variables modified: rx, rx_head, rx_fault and the RX LED event
 * flag.
 * Errors: Overflow or rearm failure sets rx_fault for main-loop handling.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* uart)
{
    if (uart != port) {
        return;
    }
    StatusLed_RxReceived();
    unsigned next = (rx_head + 1U) % RX_SIZE;
    if (!rx_fault) {
        if (next == rx_tail) {
            rx_fault = 1;
        } else {
            rx[rx_head] = rx_byte;
            rx_head = next;
        }
    }
    if (HAL_UART_Receive_IT(port, &rx_byte, 1) != HAL_OK) {
        rx_fault = 1;
    }
}

/* HAL_UART_TxCpltCallback()
 * -------------------------
 * Release the completed TX chunk for the bound UART; ignore other UARTs.
 * Runs in interrupt context.
 *
 * uart: HAL handle whose transmission completed.
 *
 * Returns: None.
 * Global variables modified: tx_tail, tx_active and the TX LED event flag.
 * Errors: None.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* uart)
{
    if (uart != port) {
        return;
    }
    StatusLed_TxComplete();
    tx_tail = (tx_tail + tx_active) % TX_SIZE;
    tx_active = 0;
}

/* HAL_UART_ErrorCallback()
 * ------------------------
 * Flag a bound-UART fault for main-loop recovery; ignore other UARTs.
 *
 * uart: HAL handle reporting the error.
 *
 * Returns: None.
 * Global variables modified: rx_fault.
 * Errors: None.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* uart)
{
    if (uart == port) {
        rx_fault = 1;
    }
}

/* number()
 * --------
 * Parse a whole signed decimal integer, rejecting empty text, trailing junk
 * and overflow.
 *
 * text: Input string; may be NULL.
 * value: Writable result pointer, unchanged on failure.
 *
 * Returns: 1 on success, otherwise 0.
 * Global variables modified: None.
 * Errors: Invalid input returns 0 without writing value.
 */
static int number(const char* text, int32_t* value)
{
    if (!text || !*text) {
        return 0;
    }
    char* end;
    errno = 0;
    long parsed = strtol(text, &end, 10);
    if (errno == ERANGE || *end || parsed < INT32_MIN || parsed > INT32_MAX) {
        return 0;
    }
    *value = (int32_t)parsed;
    return 1;
}

/* limits()
 * --------
 * Queue the raw SW1..SW4 states, including PC11.
 *
 * Returns: None.
 * Global variables modified: TX queue through reply().
 * Errors: Queue-full output is dropped and counted.
 */
static void limits(void)
{
    reply("LIMITS: SW1=%u SW2/PC11=%u SW3=%u SW4=%u (1=PRESSED)\r\n",
            Switch_IsPressed(1), Switch_IsPressed(2),
            Switch_IsPressed(3), Switch_IsPressed(4));
}

/* step_status()
 * -------------
 * Queue stepper mode, target, active speed and remaining pulse count.
 *
 * Returns: None.
 * Global variables modified: TX queue through reply().
 * Errors: Queue-full output is dropped and counted.
 */
static void step_status(void)
{
    const RobotStatus* s = Robot_GetStatus();
    const char* modes[] = {"STOPPED", "AUTO", "MANUAL"};
    reply("STEPPER: %s; TARGET: SW%u; SPEED: %lu STEPS/S; REMAINING: "
            "%lu%s\r\n",
            modes[s->step_mode], s->target, (unsigned long)s->speed,
            (unsigned long)(s->paused
                    ? s->pause_remaining : Stepper1_Remaining()),
            s->paused ? " (PAUSED)" : "");
}

/* calibration_status()
 * --------------------
 * Queue saved straight, usable ranges and true/path versus adjusted command
 * angle. Values are commands, not measurements.
 *
 * Returns: None.
 * Global variables modified: TX queue through reply().
 * Errors: Queue-full output is dropped and counted.
 */
static void calibration_status(void)
{
    const RobotStatus* s = Robot_GetStatus();
    int offset = (int)s->straight_angle - DEMO_START_ANGLE;
    reply("CALIBRATION: STRAIGHT=%u DEG; OFFSET=%+d DEG; "
            "RUN RANGE=%u..%u DEG (RESETS ON POWER-UP)\r\n",
            s->straight_angle, offset, s->straight_angle,
            s->straight_angle + DEMO_END_ANGLE - DEMO_START_ANGLE);
    reply("RUN2 ADJUSTED RANGE=%u..%u DEG; %s\r\n", s->straight_angle,
            s->straight_angle + RUN2_END_ANGLE - DEMO_START_ANGLE,
            s->straight_angle <= 125U ? "CALIBRATION VALID"
                    : "BLOCKED: CALIBRATE 0..125 FOR RUN2");
    if (s->servo_pulse_us[1] < 0) {
        reply("SERVO 2: UNCOMMANDED\r\n");
    } else {
        reply("SERVO 2: TRUE (PATH)=%d DEG; ADJUSTED (COMMAND)=%d DEG; "
                "%d US (NOT MEASURED)\r\n",
                s->servo_angle[1] - offset, s->servo_angle[1],
                s->servo_pulse_us[1]);
    }
}

/* status()
 * --------
 * Queue controller, actuator, switch, speed-profile and path diagnostics.
 *
 * Returns: None.
 * Global variables modified: TX queue through reply().
 * Errors: Queue-full output is dropped and counted.
 */
static void status(void)
{
    const RobotStatus* s = Robot_GetStatus();
    const char* states[] = {"IDLE", "RUNNING", "STOPPED"};
    reply("STATE: %s\r\nDC MOTOR: %d%%\r\nFAULT: %s\r\n"
            "PC11 RUN SWITCH: %s\r\nTX RESPONSES DROPPED: %u\r\n",
            s->paused ? "RUNNING (PAUSED)" : states[s->state], s->duty,
            s->fault == ROBOT_FAULT_SW4_STUCK
                    ? "LIMIT FAULT LATCHED (SW4 STUCK)"
                    : s->fault == ROBOT_FAULT_RUN2_LIMIT
                    ? "LIMIT FAULT LATCHED (UNEXPECTED RUN2 LIMIT)"
                    : s->fault ? "LIMIT FAULT LATCHED" : "NONE",
            Switch_IsPressed(2) ? "ON" : "OFF", tx_dropped);
    step_status();
    calibration_status();
    limits();

    reply("RUN SPEEDS: STRAIGHT=%lu; CURVE=%lu; RETURN=%lu STEPS/S\r\n",
            (unsigned long)s->straight_speed, (unsigned long)s->curve_speed,
            (unsigned long)s->return_speed);
    reply("RUN2 SPEEDS: STRAIGHT1=%lu; CURVE1=%lu; RETRACT1=%lu; "
            "CURVE2=%lu; RETURN2=%lu STEPS/S\r\n",
            (unsigned long)s->run2_speeds[0],
            (unsigned long)s->run2_speeds[1],
            (unsigned long)s->run2_speeds[2],
            (unsigned long)s->run2_speeds[3],
            (unsigned long)s->run2_speeds[4]);
    reply("PROGRAM: %s; PHASE=%s; INTERMEDIATE RETRACT=%lu STEPS\r\n",
            s->run2 ? "RUN2" : "RUN", leg(s),
            (unsigned long)RUN2_RETRACT_POSITION);
    reply("DEMO: %s; POSITION FROM SW4: %lu STEPS; SERVO 2: %d US\r\n",
            s->state != ROBOT_RUNNING
                    ? (s->run_complete ? "COMPLETE" : "INACTIVE")
                    : s->paused ? "PAUSED"
                    : s->demo_backoff ? "BACKING OFF SW4"
                    : s->demo_homing ? "HOMING SW4" : "FOLLOWING TABLE",
            (unsigned long)s->demo_position, s->servo_pulse_us[1]);
    reply("OUTWARD LIMIT: %lu STEPS\r\n",
            (unsigned long)RUN_OUTWARD_STOP_POSITION);
    reply("PATH: OUTWARD END=%lu; RETURN PULSES=%lu; "
            "RETURN TARGET=SW4; HOME ERROR=%ld STEPS\r\n",
            (unsigned long)s->outward_steps, (unsigned long)s->return_steps,
            (long)s->home_step_error);
}

/* leg()
 * -----
 * Select a stable display label for the current run phase.
 *
 * s: Non-NULL pointer to the live controller status.
 *
 * Returns: Static phase label, or "-" for an inactive run.
 * Global variables modified: None.
 * Errors: None.
 */
static const char* leg(const RobotStatus* s)
{
    if (s->state != ROBOT_RUNNING) {
        return "-";
    }
    if (s->run2 && !s->demo_homing) {
        const char* phases[] = {"CUT1", "RETRACT1", "CUT2", "RETURN2"};
        return phases[s->run2_phase];
    }
    return s->demo_backoff ? "BACKOFF" : s->demo_homing ? "HOMING"
            : s->target == 1U ? "OUT" : "RETURN";
}

/* tenths_mm()
 * -----------
 * Convert steps using the measured 398.87 steps/mm scale.
 *
 * steps: Counted distance from the coordinate origin.
 *
 * Returns: Rounded tenths of a millimetre; uses widened arithmetic.
 * Global variables modified: None.
 * Errors: None.
 */
static unsigned long tenths_mm(uint32_t steps)
{
    return (unsigned long)(((uint64_t)steps * 1000U + 19943U) / 39887U);
}

/* data_row()
 * ----------
 * Queue one CSV row in data_header column order. Pod angle is relative to
 * calibrated straight.
 *
 * s: Non-NULL controller status pointer.
 *
 * Returns: None.
 * Global variables modified: TX queue through reply().
 * Errors: Queue-full output is dropped and counted.
 */
static void data_row(const RobotStatus* s)
{
    unsigned long mm = tenths_mm(s->demo_position);
    int offset = s->servo_pulse_us[1] < 0 ? 0
            : s->servo_pulse_us[1] - (int)SERVO_ANGLE_US(s->straight_angle);
    /* Nominal pod angle from the arc start, in tenths of a degree, rounded
     * half away from zero.
     */
    int pod = (abs(offset) * 10 + (int)DEMO_US_PER_POD_DEG / 2)
            / (int)DEMO_US_PER_POD_DEG;
    reply("DATA,%lu,%s,%lu,%lu.%lu,%d,%s%d.%d,%d,%u,%u\r\n",
            (unsigned long)HAL_GetTick(), leg(s),
            (unsigned long)s->demo_position, mm / 10U, mm % 10U,
            s->servo_pulse_us[1], offset < 0 ? "-" : "",
            pod / 10, pod % 10, s->duty,
            Switch_IsPressed(1), Switch_IsPressed(4));
}

/* where()
 * -------
 * Write a bounded location description; homing/back-off have no known
 * position.
 *
 * text: Writable output buffer.
 * size: Buffer capacity in bytes.
 * s: Non-NULL controller status pointer.
 * leg_name: Valid phase-label string.
 *
 * Returns: None.
 * Global variables modified: None.
 * Errors: Output is truncated according to snprintf rules if size is too
 * small.
 */
static void where(char* text, size_t size, const RobotStatus* s,
        const char* leg_name)
{
    unsigned long mm = tenths_mm(s->demo_position);
    if (!strcmp(leg_name, "HOMING") || !strcmp(leg_name, "BACKOFF")) {
        snprintf(text, size, "DURING %s (POSITION NOT SET)", leg_name);
    } else {
        snprintf(text, size, "AT: %lu STEPS (%lu.%lu MM FROM SW4) DURING %s",
                (unsigned long)s->demo_position, mm / 10U, mm % 10U, leg_name);
    }
}

/* run_events()
 * ------------
 * Report start/pause/turn/stop/fault changes by comparing live status with
 * the previously reported state.
 *
 * Returns: None.
 * Global variables modified: was_running/paused/fault/arc, arc_done_seen,
 * last_leg, telemetry timing
 *     and TX queue.
 * Errors: Queue-full output is dropped and counted.
 */
static void run_events(void)
{
    const RobotStatus* s = Robot_GetStatus();
    char at[80];
    if (s->state == ROBOT_RUNNING) {
        if (!was_running) {
            was_running = 1;
            was_paused = 0;
            if (s->run2) {
                reply("RUN2: TWO CUTS AT 157297 STEPS; 135 THEN 145 DEG; "
                        "RETRACT TO 95000 BETWEEN CUTS; "
                        "FINAL RETURN TO SW4\r\n");
            }
            if (s->demo_backoff) {
                reply("RUN STARTED: SW4 PRESSED; BACKING OFF %u STEPS, "
                        "THEN HOMING\r\n", (unsigned)RUN_START_BACKOFF_STEPS);
            } else {
                reply("RUN STARTED: HOMING TO SW4\r\n");
            }
            if (telemetry_on) {
                reply("%s", data_header);
            }
            /* Make the first DATA row due immediately. */
            telemetry_last = HAL_GetTick() - telemetry_ms;
        }
        if (s->paused != was_paused) {
            was_paused = s->paused;
            if (s->paused) {
                where(at, sizeof at, s, leg(s));
                reply("PAUSED %s; SERVO HOLDING %d US; "
                        "send continue or stop\r\n", at, s->servo_pulse_us[1]);
            } else {
                reply("CONTINUING: %s\r\n", leg(s));
                telemetry_last = HAL_GetTick() - telemetry_ms;
            }
        }
        last_leg = leg(s);
        if (telemetry_on && !s->paused
                && (uint32_t)(HAL_GetTick() - telemetry_last) >= telemetry_ms) {
            telemetry_last = HAL_GetTick();
            data_row(s);
        }
    } else if (was_running) {
        was_running = was_paused = 0;
        if (s->run_complete) {
            reply("RUN COMPLETE: SW4 REACHED; POSITION=%lu STEPS; "
                    "SERVO HOLDING %d US\r\n",
                    (unsigned long)s->demo_position, s->servo_pulse_us[1]);
        } else {
            where(at, sizeof at, s, last_leg);
            reply("STOPPED %s; SERVO HOLDING %d US\r\n",
                    at, s->servo_pulse_us[1]);
        }
    }
    if (s->arc_test != was_arc) {
        was_arc = s->arc_test;
        if (!s->arc_test) {
            reply("SERVO ARC TEST %s: SERVO HOLDING %d US\r\n",
                    s->arc_tests_done != arc_done_seen ? "DONE" : "STOPPED",
                    s->servo_pulse_us[1]);
        }
    }
    arc_done_seen = s->arc_tests_done;
    if (s->fault && !was_fault) {
        reply("FAULT: LIMIT FAULT LATCHED (%s); ROBOT STOPPED; "
                "RESET REQUIRED\r\n",
                s->fault == ROBOT_FAULT_SW4_STUCK
                        ? "SW4 STILL PRESSED AFTER BACK-OFF"
                        : s->fault == ROBOT_FAULT_RUN2_LIMIT
                        ? "UNEXPECTED LIMIT DURING RUN2 COUNTED LEG"
                        : "SW1 AND SW4 PRESSED");
    }
    was_fault = s->fault;
}

/* result()
 * --------
 * Queue a command success or error response.
 *
 * error: NULL for success, otherwise a static error explanation.
 * success: Valid success text when error is NULL; unused otherwise.
 *
 * Returns: None.
 * Global variables modified: TX queue through reply().
 * Errors: Queue-full output is dropped and counted.
 */
static void result(const char* error, const char* success)
{
    reply(error ? "ERROR: %s\r\n" : "OK: %s\r\n", error ? error : success);
}

/* help()
 * ------
 * Queue the full command guide in bounded message fragments.
 *
 * Returns: None.
 * Global variables modified: TX queue through reply().
 * Errors: Queue-full output is dropped and counted.
 */
static void help(void)
{
    reply("%s", "available commands:\r\n"
            "Type one command and press Enter. Commands ignore letter "
            "case.\r\n"
            "Use whole numbers for angles, duty, steps and speed.\r\n"
            "\r\n"
            "GENERAL\r\n"
            "  help       Show this full command guide.\r\n"
            "  ping       Test the serial link; replies OK PONG.\r\n"
            "  status     Show robot state, stepper "
            "mode/target/speed/remaining,\r\n"
            "             signed motor duty, limits, run switch and "
            "faults.\r\n"
            "             Also reports dropped serial response fragments.\r\n"
            "  limits     Read SW1-SW4: 1=pressed, 0=released; SW2 is "
            "PC11.\r\n"
            "\r\n");
    reply("AUTONOMOUS CONTROL\r\n"
            "  run        Home to SW4 first (DC off, servo 2 at calibrated "
            "straight).\r\n"
            "             Starting on SW4: back off %u steps, then home.\r\n"
            "             Push toward SW1 (DC -100%%), reverse at 157297 "
            "steps, return to\r\n"
            "             SW4 (DC +100%%), then stop on the switch: one "
            "cycle.\r\n"
            "             Servo 2 follows the smooth foam-clearance path; "
            "CLI speed.\r\n",
            (unsigned)RUN_START_BACKOFF_STEPS);
    reply("%s", "  run2       Home, cut to 135 deg at 157297, retract to "
            "95000,\r\n"
            "             cut to 145 deg at 157297, then return to physical "
            "SW4.\r\n"
            "             Cut 1 turns at 100000; cut 2 at 95000. Calibration "
            "applies.\r\n"
            "  speed2(straight1,curve1,retract1,curve2,return2)\r\n"
            "             Five independent speeds, each 16..2000 steps/s; "
            "stop first.\r\n"
            "             Defaults: speed2(750,500,2000,800,2000).\r\n"
            "             Homing/backoff always use 2000 steps/s; cut2 "
            "curves immediately.\r\n"
            "             run2 requires calibrated straight in 0..125 "
            "degrees.\r\n");
    reply("%s", "  pause      Freeze a run where it is: stepper and DC stop, "
            "servo\r\n"
            "             holds. The run is kept; run and manual moves are "
            "refused.\r\n"
            "  continue   Resume a paused run from where it stopped, same "
            "leg.\r\n"
            "  stop       Stop DC motor and stepper where they are, cancel "
            "pending\r\n"
            "             moves/auto, hold servo angle, and enter "
            "STOPPED.\r\n"
            "             This is a software stop, not a hardware emergency "
            "stop.\r\n"
            "\r\n");
    reply("SERVO (physical servo 2 only, PB6)\r\n"
            "  servo(angle)  Set target angle from 0 to 180 degrees.\r\n"
            "                Example: servo(%d), the centre. Enables servo 2 "
            "PWM.\r\n"
            "  servo(%uu) Set pulse width, 500..2500 us (servo 2 only).\r\n"
            "  servo(status) Show target degrees and us; -1=uncommanded.\r\n"
            "                Target is not measured position; stop holds "
            "the\r\n"
            "                servo at this last target.\r\n",
            SERVO_CENTRE_ANGLE, (unsigned)SERVO_ANGLE_US(SERVO_CENTRE_ANGLE));
    reply("  servo(arc)    Sweep the run's arc: %d -> %d -> %d deg in "
            "about\r\n"
            "                %u s, then hold at %d. stop aborts it.\r\n"
            "  servo()       Show this guide without moving.\r\n"
            "                Pod fitted: stay within %uu..%uu; the keeper\r\n"
            "                allows only 23.5 deg of pod (20 us per "
            "degree).\r\n"
            "\r\n",
            DEMO_START_ANGLE, DEMO_END_ANGLE, DEMO_START_ANGLE,
            (unsigned)((2U * ARC_TEST_SWEEP_MS + ARC_TEST_HOLD_MS
                    + 500U) / 1000U), DEMO_START_ANGLE,
            (unsigned)SERVO_ANGLE_US(SERVO_CENTRE_ANGLE) - 468U,
            (unsigned)SERVO_ANGLE_US(SERVO_CENTRE_ANGLE) + 468U);
    reply("%s", "DC MOTOR\r\n"
            "  motor(duty)   Set duty from -100 to +100 percent. Positive "
            "is\r\n"
            "                forward, negative reverse, and zero stops the "
            "motor.\r\n"
            "                Examples: motor(30), motor(-30), motor(0).\r\n"
            "  motor(stop)   Stop DC motor. During run, stops the whole "
            "robot.\r\n"
            "                motor(0) has the same stopping behavior.\r\n"
            "  motor(status) Show signed duty; negative indicates "
            "reverse.\r\n"
            "  motor()       Show this guide without moving.\r\n"
            "\r\n");
    reply("%s", "STEPPER\r\n"
            "  stepper(steps)    Move a signed number of STEP pulses, then "
            "stop.\r\n"
            "                    Positive goes toward SW4; negative toward "
            "SW1.\r\n"
            "                    Range: -1000000..-1 or 1..1000000.\r\n"
            "                    Examples: stepper(20), stepper(-20).\r\n"
            "                    Stops early at its target limit. Stop any "
            "current\r\n"
            "                    stepper motion before starting a counted "
            "move.\r\n"
            "  stepper(speed,n)  Set nominal speed, 16..2000 "
            "steps/second.\r\n"
            "                    Example: stepper(speed,100). Must be "
            "stopped;\r\n"
            "                    setting speed does not start movement.\r\n");
    reply("%s",
            "  stepper(auto)     Travel continuously between SW1 and SW4.\r\n"
            "                    Replaces a counted move; leaves DC duty "
            "unchanged.\r\n"
            "  stepper(stop)     Stop stepper and cancel pending "
            "steps/auto.\r\n"
            "                    During run, stops the whole robot.\r\n"
            "  stepper(status)   Show mode, target switch, speed and steps "
            "remaining.\r\n"
            "  stepper()         Show this guide without moving.\r\n"
            "\r\n");
    reply("%s", "TELEMETRY (printed during run)\r\n"
            "  telemetry(on)   CSV DATA rows (default on, every 1000 ms):\r\n"
            "                  ms,leg,steps,mm,servo_us,pod_deg,dc,sw1,sw4.\r"
            "\n"
            "  telemetry(N)    Same, every N ms (100..10000).\r\n"
            "  telemetry(off)  No DATA rows; RUN STARTED, TURN, RUN "
            "COMPLETE,\r\n"
            "                  STOPPED AT and FAULT lines still print.\r\n"
            "\r\n");
    reply("%s", "  speed(straight,curve,return)\r\n"
            "             Set run section speeds, each 16..2000 steps/s, "
            "while stopped.\r\n"
            "             Default: speed(750,500,2000). Homing/backoff "
            "always use 2000;\r\n"
            "             curve starts at 100000 steps from SW4 first "
            "touch;\r\n"
            "             return uses its own speed all the way home.\r\n"
            "             stepper(speed,n) sets all three speeds to n.\r\n"
            "\r\n");
    reply("%s", "CALIBRATION\r\n"
            "  straight()       Move servo 2 to calibrated straight (default "
            "90 degrees).\r\n"
            "                   Keeps calibration; same restrictions as "
            "manual servo commands.\r\n"
            "  calibrate(angle) Set straight to an absolute servo angle, "
            "0..135.\r\n"
            "                   Shifts the full 45-degree run/arc sweep. "
            "Stop all motion first.\r\n"
            "                   Manual servo commands stay absolute. Resets "
            "on power-up.\r\n"
            "  calibrate(90)    Restore the default straight angle.\r\n"
            "  calibrate(status) Show calibration and true/path vs "
            "adjusted/command angles.\r\n"
            "                   These are commands, not measured "
            "positions.\r\n"
            "\r\n");
    reply("%s", "MOTION RULES\r\n"
            "  PC11/SW2: OFF->ON starts run2; ON->OFF stops (30 ms "
            "debounce).\r\n"
            "  After completion/stop, cycle OFF then ON to run again.\r\n"
            "  ON at reset requires OFF then ON before switch-started "
            "motion.\r\n"
            "  No startup interlock; UART commands work immediately.\r\n"
            "  During run, send stop before individual movement/speed "
            "commands.\r\n"
            "  SW1 and SW4 pressed together latch a fault: correct cause and "
            "reset.\r\n"
            "  SW4 still pressed after the run-start back-off also latches "
            "one.\r\n"
            "  IDLE means no full autonomous program; individual motors may "
            "move.\r\n"
            "  After stop, another accepted movement command can resume "
            "motion.\r\n"
            "\r\n");
}

/* trim()
 * ------
 * Strip leading/trailing spaces and tabs from a command fragment in place.
 *
 * text: Writable, NUL-terminated input.
 *
 * Returns: Pointer to the first non-space character; may point to the
 * terminator.
 * Global variables modified: None.
 * Errors: None.
 */
static char* trim(char* text)
{
    while (*text == ' ' || *text == '\t') {
        ++text;
    }
    char* end = text + strlen(text);
    while (end > text && (end[-1] == ' ' || end[-1] == '\t')) {
        --end;
    }
    *end = '\0';
    return text;
}

/* command()
 * ---------
 * Parse and dispatch the completed line in main context. Validate full
 * arguments before issuing actuator requests.
 *
 * Returns: None.
 * Global variables modified: line, controller state through commands,
 * telemetry options and TX queue.
 * Errors: Invalid syntax queues an error; the shared syntax_error label
 *     keeps recovery consistent.
 */
static void command(void)
{
    char* words[7];
    unsigned count = 1;
    char* text = trim(line);
    if (!*text) {
        return;
    }
    for (char* p = text; *p; ++p) {
        *p = (char)toupper((unsigned char)*p);
    }

    words[0] = text;
    char* open = strchr(text, '(');
    if (open) {
        *open = '\0';
        words[0] = trim(text);
        char* args = open + 1;
        char* close = strchr(args, ')');
        if (!close || *trim(close + 1) || strchr(args, '(')) {
            goto syntax_error;
        }
        *close = '\0';
        args = trim(args);
        if (!strcmp(words[0], "STRAIGHT")) {
            if (*args) {
                goto syntax_error;
            }
            result(Robot_SetServo(2U, Robot_GetStatus()->straight_angle),
                    "SERVO AT STRAIGHT TARGET");
            calibration_status();
            return;
        }
        if (strcmp(words[0], "SERVO") && strcmp(words[0], "MOTOR")
                && strcmp(words[0], "STEPPER")
                && strcmp(words[0], "TELEMETRY")
                && strcmp(words[0], "SPEED") && strcmp(words[0], "SPEED2")
                && strcmp(words[0], "CALIBRATE")) {
            goto syntax_error;
        }
        if (!*args) {
            help();
            return;
        }
        while (1) {
            if (count == 7) {
                goto syntax_error;
            }
            char* comma = strchr(args, ',');
            if (comma) {
                *comma = '\0';
            }
            words[count] = trim(args);
            if (!*words[count]) {
                goto syntax_error;
            }
            ++count;
            if (!comma) {
                break;
            }
            args = comma + 1;
        }
    } else if (strpbrk(text, "() ,\t")) {
        goto syntax_error;
    }

    int32_t a, b, c;
    if (!strcmp(words[0], "PING") && count == 1) {
        reply("OK PONG\r\n");
    } else if (!strcmp(words[0], "HELP") && count == 1) {
        help();
    } else if (!strcmp(words[0], "STATUS") && count == 1) {
        status();
    } else if (!strcmp(words[0], "LIMITS") && count == 1) {
        limits();
    } else if (!strcmp(words[0], "STOP") && count == 1) {
        Robot_Stop();
        result(NULL, "ROBOT STOPPED\r\nDC MOTOR: OFF\r\nSTEPPER: STOPPED");
    } else if (!strcmp(words[0], "RUN") && count == 1) {
        result(Robot_StartAuto(),
                "AUTONOMOUS RUN ACTIVE; use status for phase and position");
    } else if (!strcmp(words[0], "RUN2") && count == 1) {
        result(Robot_StartAuto2(),
                "AUTONOMOUS RUN2 ACTIVE; use status for phase and position");
    } else if (!strcmp(words[0], "PAUSE") && count == 1) {
        result(Robot_Pause(), "RUN PAUSED; send continue or stop");
    } else if (!strcmp(words[0], "CONTINUE") && count == 1) {
        result(Robot_Continue(), "RUN CONTINUING");
    } else if (!strcmp(words[0], "CALIBRATE")) {
        if (count == 2 && number(words[1], &a)) {
            result(Robot_Calibrate((unsigned)a), "STRAIGHT CALIBRATION SET");
        } else if (count == 2 && !strcmp(words[1], "STATUS")) {
            /* The common calibration report follows command dispatch. */
        } else {
            reply("ERROR: use calibrate(0..135) or calibrate(status)\r\n");
        }
    } else if (!strcmp(words[0], "SPEED")) {
        if (count == 4 && number(words[1], &a) && number(words[2], &b)
                && number(words[3], &c)) {
            result(Robot_SetRunSpeeds((uint32_t)a, (uint32_t)b, (uint32_t)c),
                    "RUN SPEEDS SET (STRAIGHT, CURVE, RETURN)");
        } else {
            reply("ERROR: use speed(straight,curve,return), "
                    "each 16..2000 steps/s\r\n");
        }
    } else if (!strcmp(words[0], "SPEED2")) {
        uint32_t speeds[5];
        if (count != 6) {
            goto syntax_error;
        }
        for (unsigned i = 0; i < 5; ++i) {
            if (!number(words[i + 1], &a)) {
                goto syntax_error;
            }
            speeds[i] = (uint32_t)a;
        }
        result(Robot_SetRun2Speeds(speeds),
                "RUN2 SPEEDS SET (STRAIGHT1, CURVE1, RETRACT1, "
                "CURVE2, RETURN2)");
    } else if (!strcmp(words[0], "STEPPER")) {
        if (count == 2 && !strcmp(words[1], "STATUS")) {
            step_status();
        } else if (count == 2 && !strcmp(words[1], "STOP")) {
            Robot_StopStepper();
            result(NULL, "STEPPER STOPPED (RUN CANCELLED IF ACTIVE)");
        } else if (count == 2 && !strcmp(words[1], "AUTO")) {
            result(Robot_StepperAuto(), "STEPPER AUTO");
        } else if (count == 2 && number(words[1], &a)) {
            result(Robot_Move(a), "STEPPER MOVE STARTED");
        } else if (count == 3 && !strcmp(words[1], "SPEED")
                && number(words[2], &a)) {
            result(Robot_SetSpeed((uint32_t)a), "STEPPER SPEED SET");
        } else {
            reply("ERROR: use stepper(steps), stepper(speed,16..2000), "
                    "stepper(auto|stop|status)\r\n");
        }
    } else if (!strcmp(words[0], "MOTOR")) {
        if (count == 2 && !strcmp(words[1], "STATUS")) {
            reply("DC MOTOR: %d%%\r\n", Robot_GetStatus()->duty);
        } else if (count == 2 && !strcmp(words[1], "STOP")) {
            Robot_StopMotor();
            result(NULL, "MOTOR STOPPED (RUN CANCELLED IF ACTIVE)");
        } else if (count == 2 && number(words[1], &a)) {
            result(Robot_SetMotor((int)a), "MOTOR DUTY SET");
        } else {
            reply("ERROR: use motor(-100..100), motor(stop), "
                    "motor(status)\r\n");
        }
    } else if (!strcmp(words[0], "SERVO")) {
        if (count == 2 && !strcmp(words[1], "STATUS")) {
            reply("SERVO 2 TARGET: %d DEG (NOMINAL), %d US "
                    "(-1=UNCOMMANDED; stop HOLDS IT)%s\r\n",
                    Robot_GetStatus()->servo_angle[1],
                    Robot_GetStatus()->servo_pulse_us[1],
                    Robot_GetStatus()->arc_test ? "; ARC TEST RUNNING" : "");
        } else if (count == 2 && !strcmp(words[1], "ARC")) {
            const char* error = Robot_ServoArcTest();
            if (error) {
                result(error, NULL);
            } else {
                reply("OK: SERVO ARC TEST %d -> %d -> %d DEG "
                        "(%u -> %u US), ABOUT %u S "
                        "(TRUE PATH; SEE ADJUSTED RANGE BELOW); "
                        "stop aborts\r\n",
                        DEMO_START_ANGLE, DEMO_END_ANGLE, DEMO_START_ANGLE,
                        (unsigned)DEMO_START_PULSE_US,
                        (unsigned)DEMO_END_PULSE_US,
                        (unsigned)((2U * ARC_TEST_SWEEP_MS
                                + ARC_TEST_HOLD_MS + 500U) / 1000U));
            }
        } else if (count == 2 && strlen(words[1]) > 1
                && words[1][strlen(words[1]) - 1] == 'U') {
            words[1][strlen(words[1]) - 1] = '\0';
            if (number(words[1], &a)) {
                result(Robot_SetServoPulseUs(2U, (unsigned)a),
                        "SERVO 2 PULSE SET");
            } else {
                reply("ERROR: use servo(500u..2500u)\r\n");
            }
        } else if (count == 2 && number(words[1], &a)) {
            result(Robot_SetServo(2U, (unsigned)a), "SERVO 2 TARGET SET");
        } else {
            reply("ERROR: use servo(0..180), servo(500u..2500u), "
                    "servo(arc) or servo(status)\r\n");
        }
    } else if (!strcmp(words[0], "TELEMETRY")) {
        if (count == 2 && !strcmp(words[1], "OFF")) {
            telemetry_on = 0;
            result(NULL, "TELEMETRY OFF");
        } else if (count == 2 && (!strcmp(words[1], "ON")
                || (number(words[1], &a) && a >= 100 && a <= 10000))) {
            if (strcmp(words[1], "ON")) {
                telemetry_ms = (uint32_t)a;
            }
            if (!telemetry_on && was_running) {
                reply("%s", data_header);
            }
            telemetry_on = 1;
            reply("OK: TELEMETRY ON; DATA ROW EVERY %lu MS DURING RUN\r\n",
                    (unsigned long)telemetry_ms);
        } else {
            reply("ERROR: use telemetry(on), telemetry(off), "
                    "telemetry(100..10000)\r\n");
        }
    } else {
syntax_error:
        reply("ERROR: UNKNOWN COMMAND OR INVALID PARAMETERS\r\n"
                "Type help for available commands\r\n");
    }

    if (!strcmp(words[0], "CALIBRATE") || !strcmp(words[0], "SERVO")
            || !strcmp(words[0], "RUN") || !strcmp(words[0], "RUN2")
            || !strcmp(words[0], "STOP") || !strcmp(words[0], "PAUSE")
            || !strcmp(words[0], "CONTINUE")) {
        calibration_status();
    }
}

/* Serial_Update()
 * ---------------
 * Process a bounded batch of input, run events and queued output in main.
 * Never wait for UART; commands may change controller state.
 *
 * Returns: None.
 * Global variables modified: RX/TX queues, parser/event state and controller
 * state via commands.
 * Errors: RX faults/overlong lines stop motion and discard damaged input;
 *     full TX queues drop responses.
 */
void Serial_Update(void)
{
    if (rx_fault) {
        Robot_Stop();
        /* Discard damaged input and resynchronize at a fresh terminator.
           Never interpret a truncated command as a movement command. */
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        HAL_UART_AbortReceive(port);
        __HAL_UART_CLEAR_FLAG(port, UART_CLEAR_OREF | UART_CLEAR_NEF
                | UART_CLEAR_FEF | UART_CLEAR_PEF);
        __HAL_UART_SEND_REQ(port, UART_RXDATA_FLUSH_REQUEST);
        rx_head = rx_tail = 0;
        rx_fault = 0;
        line_length = 0;
        discard_line = 1;
        if (HAL_UART_Receive_IT(port, &rx_byte, 1) != HAL_OK) {
            rx_fault = 1;
        }
        __set_PRIMASK(mask);
        reply("ERROR: UART RECEIVE ERROR/OVERFLOW; ROBOT STOPPED; SEND "
                "NEWLINE THEN RETRY\r\n");
    }
    /* Bound each visit so continuous serial traffic cannot starve safety. */
    for (unsigned budget = 0; budget < 32 && rx_tail != rx_head && !rx_fault;
            ++budget) {
        uint8_t ch = rx[rx_tail];
        rx_tail = (rx_tail + 1U) % RX_SIZE;
        if (ch == '\r' || ch == '\n') {
            if (!discard_line && line_length) {
                line[line_length] = '\0';
                Robot_Update();
                command();
                /* Report each command, including lines received together. */
                run_events();
            }
            line_length = discard_line = 0;
        } else if (!discard_line) {
            if ((ch < 32 && ch != '\t') || ch > 126
                    || line_length >= LINE_SIZE - 1U) {
                discard_line = 1;
                line_length = 0;
                Robot_Stop();
                reply("ERROR: INVALID/OVERLONG LINE; ROBOT STOPPED; LINE "
                        "DISCARDED\r\n");
            } else {
                line[line_length++] = (char)ch;
            }
        }
    }
    const RobotStatus* s = Robot_GetStatus();
    if (reported_turn != s->turn_number) {
        reported_turn = s->turn_number;
        if (s->run2) {
            reply("RUN2 TURN: POSITION=%lu; SERVO=%u US; NEXT=%s; "
                    "TARGET=%s\r\n",
                    (unsigned long)s->turn_position, s->turn_pulse_us, leg(s),
                    s->run2_phase == RUN2_RETRACT1
                            ? "95000 FROM SW4 FIRST TOUCH"
                            : s->run2_phase == RUN2_RETURN2
                            ? "PHYSICAL SW4" : "157297 FROM SW4 FIRST TOUCH");
        } else {
            reply("TURN: %s; POSITION=%lu; SERVO=%u US; NEXT=SW%u; "
                    "DC=%d%%; RETURN TARGET=SW4\r\n",
                    s->turn_at_position ? "OUTWARD LIMIT"
                            : s->turn_switch == 1U ? "SW1" : "SW4",
                    (unsigned long)s->turn_position, s->turn_pulse_us,
                    s->target, s->duty);
        }
    }
    run_events();
    transmit();
}
