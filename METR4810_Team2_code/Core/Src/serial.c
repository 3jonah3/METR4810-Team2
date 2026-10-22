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
static UART_HandleTypeDef *port;
static uint8_t rx_byte, tx[TX_SIZE];
/* Both data and indices are volatile: publish each byte before the ISR head. */
static volatile uint8_t rx[RX_SIZE];
static volatile unsigned rx_head, rx_tail, rx_fault;
static volatile unsigned tx_tail, tx_active;
static unsigned tx_head, tx_dropped;
static char line[LINE_SIZE];
static unsigned line_length, discard_line;
static uint32_t reported_turn;
/* Run feedback: CSV DATA rows while running, plus event lines when a run starts,
   stops or faults (button and fault stops are otherwise silent). */
static uint8_t telemetry_on, was_running, was_fault, was_paused;
static uint32_t telemetry_ms, telemetry_last;
static const char *last_leg;
static const char data_header[] = "DATA,ms,leg,steps,mm,servo_us,pod_deg,dc,sw1,sw4\r\n";
static void help(void);

/* Main-loop producer, interrupt consumer. Never wait for transmission. */
static void reply(const char *format, ...)
{
    char message[640];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(message, sizeof message, format, args);
    va_end(args);
    if (length < 0 || (unsigned)length >= sizeof message) { ++tx_dropped; return; }
    unsigned free_bytes = (tx_tail + TX_SIZE - tx_head - 1U) % TX_SIZE;
    if ((unsigned)length > free_bytes) { ++tx_dropped; return; }
    for (int i = 0; i < length; ++i) {
        tx[tx_head] = (uint8_t)message[i];
        tx_head = (tx_head + 1U) % TX_SIZE;
    }
}

/* Give the UART the next contiguous chunk of the ring, if it is idle. */
static void transmit(void)
{
    if (!tx_active && tx_head != tx_tail) {
        unsigned count = tx_head > tx_tail ? tx_head - tx_tail : TX_SIZE - tx_tail;
        tx_active = count;
        if (HAL_UART_Transmit_IT(port, &tx[tx_tail], (uint16_t)count) != HAL_OK)
            tx_active = 0;
    }
}

void Serial_Init(UART_HandleTypeDef *uart)
{
    port = uart;
    rx_head = rx_tail = rx_fault = 0;
    tx_head = tx_tail = tx_active = tx_dropped = 0;
    line_length = discard_line = 0;
    reported_turn = Robot_GetStatus()->turn_number;
    telemetry_on = 1;
    telemetry_ms = 1000U;
    was_running = was_paused = 0;
    was_fault = 0; /* A fault latched during Robot_Init still gets its FAULT line. */
    last_leg = "-";
    if (HAL_UART_Receive_IT(port, &rx_byte, 1) != HAL_OK) rx_fault = 1;
    reply("\r\nWelcome to METR4810 Team 2 CLI\r\n\r\n ,_,\r\n(O,O)\r\n(   )\r\n-\"-\"------\r\n\r\nREADY: IDLE; type help\r\n\r\n");
    help();
}

/* Interrupt: store one received byte and re-arm reception. Overflow sets rx_fault. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != port) return;
    StatusLed_RxReceived();
    unsigned next = (rx_head + 1U) % RX_SIZE;
    if (!rx_fault) {
        if (next == rx_tail) rx_fault = 1;
        else { rx[rx_head] = rx_byte; rx_head = next; }
    }
    if (HAL_UART_Receive_IT(port, &rx_byte, 1) != HAL_OK) rx_fault = 1;
}

/* Interrupt: release the chunk that has just been sent. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != port) return;
    StatusLed_TxComplete();
    tx_tail = (tx_tail + tx_active) % TX_SIZE;
    tx_active = 0;
}

/* Interrupt: framing, noise or overrun. Serial_Update stops the robot and resyncs. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == port) rx_fault = 1;
}

/* Whole-string decimal integer. Returns 0 for empty text, trailing junk or overflow. */
static int number(const char *text, int32_t *value)
{
    if (!text || !*text) return 0;
    char *end;
    errno = 0;
    long parsed = strtol(text, &end, 10);
    if (errno == ERANGE || *end || parsed < INT32_MIN || parsed > INT32_MAX) return 0;
    *value = (int32_t)parsed;
    return 1;
}

/* Reports for the limits, stepper(status) and status commands. */
static void limits(void)
{
    reply("LIMITS: SW1=%u SW2/PC11=%u SW3=%u SW4=%u (1=PRESSED)\r\n",
          Switch_IsPressed(1), Switch_IsPressed(2), Switch_IsPressed(3), Switch_IsPressed(4));
}

static void step_status(void)
{
    const RobotStatus *s = Robot_GetStatus();
    const char *modes[] = {"STOPPED", "AUTO", "MANUAL"};
    reply("STEPPER: %s; TARGET: SW%u; SPEED: %lu STEPS/S; REMAINING: %lu%s\r\n",
          modes[s->step_mode], s->target, (unsigned long)s->speed,
          (unsigned long)(s->paused ? s->pause_remaining : Stepper1_Remaining()),
          s->paused ? " (PAUSED)" : "");
}

static void status(void)
{
    const RobotStatus *s = Robot_GetStatus();
    const char *states[] = {"IDLE", "RUNNING", "STOPPED"};
    reply("STATE: %s\r\nDC MOTOR: %d%%\r\nFAULT: %s\r\nPC11 RUN BUTTON: %s\r\nTX RESPONSES DROPPED: %u\r\n",
          s->paused ? "RUNNING (PAUSED)" : states[s->state], s->duty, s->fault == ROBOT_FAULT_SW4_STUCK ? "LIMIT FAULT LATCHED (SW4 STUCK)" :
          s->fault ? "LIMIT FAULT LATCHED" : "NONE",
          Switch_IsPressed(2) ? "PRESSED" : "RELEASED", tx_dropped);
    step_status();
    limits();
    reply("DEMO: %s; POSITION FROM SW4: %lu STEPS; SERVO 2: %d US\r\n",
          s->state != ROBOT_RUNNING ? (s->run_complete ? "COMPLETE" : "INACTIVE") :
          s->paused ? "PAUSED" : s->demo_backoff ? "BACKING OFF SW4" :
          s->demo_homing ? "HOMING SW4" : "FOLLOWING TABLE",
          (unsigned long)s->demo_position, s->servo_pulse_us[1]);
    reply("PATH: OUTWARD END=%lu; RETURN PULSES=%lu; LAST HOME ERROR=%ld STEPS\r\n",
          (unsigned long)s->outward_steps, (unsigned long)s->return_steps, (long)s->home_step_error);
}

/* Name of the current run phase, as used in DATA rows and event lines. */
static const char *leg(const RobotStatus *s)
{
    if (s->state != ROBOT_RUNNING) return "-";
    return s->demo_backoff ? "BACKOFF" : s->demo_homing ? "HOMING" : s->target == 1U ? "OUT" : "RETURN";
}

/* Tenths of a mm from SW4 at the measured 398.87 steps/mm. */
static unsigned long tenths_mm(uint32_t steps)
{
    return (unsigned long)(((uint64_t)steps * 1000U + 19943U) / 39887U);
}

/* One CSV row matching data_header. Values are commanded or counted, not measured. */
static void data_row(const RobotStatus *s)
{
    unsigned long mm = tenths_mm(s->demo_position);
    int offset = s->servo_pulse_us[1] < 0 ? 0 : s->servo_pulse_us[1] - (int)DEMO_STRAIGHT_PULSE_US;
    /* Nominal pod angle in tenths of a degree, rounded half away from zero. */
    int pod = (abs(offset) * 10 + (int)DEMO_US_PER_POD_DEG / 2) / (int)DEMO_US_PER_POD_DEG;
    reply("DATA,%lu,%s,%lu,%lu.%lu,%d,%s%d.%d,%d,%u,%u\r\n",
          (unsigned long)HAL_GetTick(), leg(s), (unsigned long)s->demo_position, mm / 10U, mm % 10U,
          s->servo_pulse_us[1], offset < 0 ? "-" : "", pod / 10, pod % 10, s->duty,
          Switch_IsPressed(1), Switch_IsPressed(4));
}

/* Position text for an event line; homing and back-off have no position yet. */
static void where(char *text, size_t size, const RobotStatus *s, const char *leg_name)
{
    unsigned long mm = tenths_mm(s->demo_position);
    if (!strcmp(leg_name, "HOMING") || !strcmp(leg_name, "BACKOFF"))
        snprintf(text, size, "DURING %s (POSITION NOT SET)", leg_name);
    else
        snprintf(text, size, "AT: %lu STEPS (%lu.%lu MM FROM SW4) DURING %s",
                 (unsigned long)s->demo_position, mm / 10U, mm % 10U, leg_name);
}

/* Print run feedback by comparing the robot state with what was last reported. It
   sees button and fault stops too, which never pass through command(). */
static void run_events(void)
{
    const RobotStatus *s = Robot_GetStatus();
    char at[80];
    if (s->state == ROBOT_RUNNING) {
        if (!was_running) {
            was_running = 1;
            was_paused = 0;
            if (s->demo_backoff)
                reply("RUN STARTED: SW4 PRESSED; BACKING OFF %u STEPS, THEN HOMING\r\n",
                      (unsigned)RUN_START_BACKOFF_STEPS);
            else reply("RUN STARTED: HOMING TO SW4\r\n");
            if (telemetry_on) reply("%s", data_header);
            telemetry_last = HAL_GetTick() - telemetry_ms; /* First row straight away. */
        }
        if (s->paused != was_paused) {
            was_paused = s->paused;
            if (s->paused) {
                where(at, sizeof at, s, leg(s));
                reply("PAUSED %s; SERVO HOLDING %d US; send continue or stop\r\n", at, s->servo_pulse_us[1]);
            } else {
                reply("CONTINUING: %s\r\n", leg(s));
                telemetry_last = HAL_GetTick() - telemetry_ms; /* Row straight away. */
            }
        }
        last_leg = leg(s);
        if (telemetry_on && !s->paused && (uint32_t)(HAL_GetTick() - telemetry_last) >= telemetry_ms) {
            telemetry_last = HAL_GetTick();
            data_row(s);
        }
    } else if (was_running) {
        was_running = was_paused = 0;
        if (s->run_complete)
            reply("RUN COMPLETE: BACK AT SW4; HOME ERROR=%ld STEPS; SERVO HOLDING %d US\r\n",
                  (long)s->home_step_error, s->servo_pulse_us[1]);
        else {
            where(at, sizeof at, s, last_leg);
            reply("STOPPED %s; SERVO HOLDING %d US\r\n", at, s->servo_pulse_us[1]);
        }
    }
    if (s->fault && !was_fault)
        reply("FAULT: LIMIT FAULT LATCHED (%s); ROBOT STOPPED; RESET REQUIRED\r\n",
              s->fault == ROBOT_FAULT_SW4_STUCK ? "SW4 STILL PRESSED AFTER BACK-OFF" : "SW1 AND SW4 PRESSED");
    was_fault = s->fault;
}

/* OK or ERROR reply for a command that returns NULL on success. */
static void result(const char *error, const char *success)
{
    reply(error ? "ERROR: %s\r\n" : "OK: %s\r\n", error ? error : success);
}

static void help(void)
{
    reply("%s", "available commands:\r\nType one command and press Enter. Commands ignore letter case.\r\nUse whole numbers for angles, duty, steps and speed.\r\n\r\nGENERAL\r\n  help       Show this full command guide.\r\n  ping       Test the serial link; replies OK PONG.\r\n  status     Show robot state, stepper mode/target/speed/remaining,\r\n             signed motor duty, limits, run button and faults.\r\n             Also reports dropped serial response fragments.\r\n  limits     Read SW1-SW4: 1=pressed, 0=released; SW2 is PC11.\r\n\r\n");
    reply("AUTONOMOUS CONTROL\r\n  run        Home to SW4 first (DC off, servo 2 at 1500 us, 90 deg).\r\n             Starting on SW4: back off %u steps, then home.\r\n             Push toward SW1 (DC -100%%), reverse at SW1, return to\r\n             SW4 (DC +100%%) and stop there: one cycle per run.\r\n             Servo 2 follows the CAD steering table; CLI speed.\r\n",
          (unsigned)RUN_START_BACKOFF_STEPS);
    reply("%s", "  pause      Freeze a run where it is: stepper and DC stop, servo\r\n             holds. The run is kept; run and manual moves are refused.\r\n  continue   Resume a paused run from where it stopped, same leg.\r\n  stop       Stop DC motor and stepper where they are, cancel pending\r\n             moves/auto, hold servo angle, and enter STOPPED.\r\n             This is a software stop, not a hardware emergency stop.\r\n\r\n");
    reply("%s", "SERVO (physical servo 2 only, PB6)\r\n  servo(angle)  Set target angle from 0 to 180 degrees.\r\n                Example: servo(90). Enables servo 2 PWM only.\r\n  servo(1500u) Set pulse width, 500..2500 us (servo 2 only).\r\n  servo(status) Show target degrees and us; -1=uncommanded.\r\n                Target is not measured position; stop holds the\r\n                servo at this last target.\r\n  servo()       Show this guide without moving.\r\n                Pod fitted: stay within 1032u..1968u; the keeper\r\n                allows only 23.5 deg of pod (20 us per degree).\r\n\r\n");
    reply("%s", "DC MOTOR\r\n  motor(duty)   Set duty from -100 to +100 percent. Positive is\r\n                forward, negative reverse, and zero stops the motor.\r\n                Examples: motor(30), motor(-30), motor(0).\r\n  motor(stop)   Stop DC motor. During run, stops the whole robot.\r\n                motor(0) has the same stopping behavior.\r\n  motor(status) Show signed duty; negative indicates reverse.\r\n  motor()       Show this guide without moving.\r\n\r\n");
    reply("%s", "STEPPER\r\n  stepper(steps)    Move a signed number of STEP pulses, then stop.\r\n                    Positive goes toward SW4; negative toward SW1.\r\n                    Range: -1000000..-1 or 1..1000000.\r\n                    Examples: stepper(20), stepper(-20).\r\n                    Stops early at its target limit. Stop any current\r\n                    stepper motion before starting a counted move.\r\n  stepper(speed,n)  Set nominal speed, 16..2000 steps/second.\r\n                    Example: stepper(speed,100). Must be stopped;\r\n                    setting speed does not start movement.\r\n");
    reply("%s", "  stepper(auto)     Travel continuously between SW1 and SW4.\r\n                    Replaces a counted move; leaves DC duty unchanged.\r\n  stepper(stop)     Stop stepper and cancel pending steps/auto.\r\n                    During run, stops the whole robot.\r\n  stepper(status)   Show mode, target switch, speed and steps remaining.\r\n  stepper()         Show this guide without moving.\r\n\r\n");
    reply("%s", "TELEMETRY (printed during run)\r\n  telemetry(on)   CSV DATA rows (default on, every 1000 ms):\r\n                  ms,leg,steps,mm,servo_us,pod_deg,dc,sw1,sw4.\r\n  telemetry(N)    Same, every N ms (100..10000).\r\n  telemetry(off)  No DATA rows; RUN STARTED, TURN, RUN COMPLETE,\r\n                  STOPPED AT and FAULT lines still print.\r\n\r\n");
    reply("%s", "MOTION RULES\r\n  PC11/SW2: press to run, press again to stop (30 ms debounce).\r\n  Release between presses; held at reset requires release first.\r\n  No startup interlock; UART commands work immediately.\r\n  During run, send stop before individual movement/speed commands.\r\n  SW1 and SW4 pressed together latch a fault: correct cause and reset.\r\n  SW4 still pressed after the run-start back-off also latches one.\r\n  IDLE means no full autonomous program; individual motors may move.\r\n  After stop, another accepted movement command can resume motion.\r\n\r\n");
}

/* Strip leading and trailing spaces and tabs in place. */
static char *trim(char *text)
{
    while (*text == ' ' || *text == '\t') ++text;
    char *end = text + strlen(text);
    while (end > text && (end[-1] == ' ' || end[-1] == '\t')) --end;
    *end = '\0';
    return text;
}

/* Parse and run one line: NAME or NAME(arg) or NAME(arg,arg), any letter case.
   Anything else gets a syntax error and nothing moves. */
static void command(void)
{
    char *words[3];
    unsigned count = 1;
    char *text = trim(line);
    if (!*text) return;
    for (char *p = text; *p; ++p) *p = (char)toupper((unsigned char)*p);
    words[0] = text;
    char *open = strchr(text, '(');
    if (open) {
        *open = '\0';
        words[0] = trim(text);
        char *args = open + 1;
        char *close = strchr(args, ')');
        if (!close || *trim(close + 1) || strchr(args, '(')) goto syntax_error;
        *close = '\0';
        args = trim(args);
        if (strcmp(words[0], "SERVO") && strcmp(words[0], "MOTOR") &&
            strcmp(words[0], "STEPPER") && strcmp(words[0], "TELEMETRY")) goto syntax_error;
        if (!*args) { help(); return; }
        char *comma = strchr(args, ',');
        if (comma) {
            *comma = '\0';
            words[2] = trim(comma + 1);
            if (!*words[2] || strchr(words[2], ',')) goto syntax_error;
            count = 3;
        } else count = 2;
        words[1] = trim(args);
        if (!*words[1]) goto syntax_error;
    } else if (strpbrk(text, "() ,\t")) goto syntax_error;
    int32_t a;
    if (!strcmp(words[0], "PING") && count == 1) reply("OK PONG\r\n");
    else if (!strcmp(words[0], "HELP") && count == 1) help();
    else if (!strcmp(words[0], "STATUS") && count == 1) status();
    else if (!strcmp(words[0], "LIMITS") && count == 1) limits();
    else if (!strcmp(words[0], "STOP") && count == 1) {
        Robot_Stop(); result(NULL, "ROBOT STOPPED\r\nDC MOTOR: OFF\r\nSTEPPER: STOPPED");
    } else if (!strcmp(words[0], "RUN") && count == 1)
        result(Robot_StartAuto(), "AUTONOMOUS RUN ACTIVE; use status for phase and position");
    else if (!strcmp(words[0], "PAUSE") && count == 1)
        result(Robot_Pause(), "RUN PAUSED; send continue or stop");
    else if (!strcmp(words[0], "CONTINUE") && count == 1)
        result(Robot_Continue(), "RUN CONTINUING");
    else if (!strcmp(words[0], "STEPPER")) {
        if (count == 2 && !strcmp(words[1], "STATUS")) step_status();
        else if (count == 2 && !strcmp(words[1], "STOP")) {
            Robot_StopStepper(); result(NULL, "STEPPER STOPPED (RUN CANCELLED IF ACTIVE)");
        } else if (count == 2 && !strcmp(words[1], "AUTO")) result(Robot_StepperAuto(), "STEPPER AUTO");
        else if (count == 2 && number(words[1], &a)) result(Robot_Move(a), "STEPPER MOVE STARTED");
        else if (count == 3 && !strcmp(words[1], "SPEED") && number(words[2], &a))
            result(Robot_SetSpeed((uint32_t)a), "STEPPER SPEED SET");
        else reply("ERROR: use stepper(steps), stepper(speed,16..2000), stepper(auto|stop|status)\r\n");
    } else if (!strcmp(words[0], "MOTOR")) {
        if (count == 2 && !strcmp(words[1], "STATUS")) reply("DC MOTOR: %d%%\r\n", Robot_GetStatus()->duty);
        else if (count == 2 && !strcmp(words[1], "STOP")) {
            Robot_StopMotor(); result(NULL, "MOTOR STOPPED (RUN CANCELLED IF ACTIVE)");
        } else if (count == 2 && number(words[1], &a)) result(Robot_SetMotor((int)a), "MOTOR DUTY SET");
        else reply("ERROR: use motor(-100..100), motor(stop), motor(status)\r\n");
    } else if (!strcmp(words[0], "SERVO")) {
        if (count == 2 && !strcmp(words[1], "STATUS"))
            reply("SERVO 2 TARGET: %d DEG (NOMINAL), %d US (-1=UNCOMMANDED; stop HOLDS IT)\r\n",
                  Robot_GetStatus()->servo_angle[1], Robot_GetStatus()->servo_pulse_us[1]);
        else if (count == 2 && strlen(words[1]) > 1 && words[1][strlen(words[1]) - 1] == 'U') {
            words[1][strlen(words[1]) - 1] = '\0';
            if (number(words[1], &a))
                result(Robot_SetServoPulseUs(2U, (unsigned)a), "SERVO 2 PULSE SET");
            else reply("ERROR: use servo(500u..2500u)\r\n");
        }
        else if (count == 2 && number(words[1], &a))
            result(Robot_SetServo(2U, (unsigned)a), "SERVO 2 TARGET SET");
        else reply("ERROR: use servo(0..180), servo(500u..2500u), or servo(status)\r\n");
    } else if (!strcmp(words[0], "TELEMETRY")) {
        if (count == 2 && !strcmp(words[1], "OFF")) {
            telemetry_on = 0; result(NULL, "TELEMETRY OFF");
        } else if (count == 2 && (!strcmp(words[1], "ON") ||
                   (number(words[1], &a) && a >= 100 && a <= 10000))) {
            if (strcmp(words[1], "ON")) telemetry_ms = (uint32_t)a;
            if (!telemetry_on && was_running) reply("%s", data_header);
            telemetry_on = 1;
            reply("OK: TELEMETRY ON; DATA ROW EVERY %lu MS DURING RUN\r\n", (unsigned long)telemetry_ms);
        } else reply("ERROR: use telemetry(on), telemetry(off), telemetry(100..10000)\r\n");
    } else {
syntax_error:
        reply("ERROR: UNKNOWN COMMAND OR INVALID PARAMETERS\r\nType help for available commands\r\n");
    }
}

void Serial_Update(void)
{
    if (rx_fault) {
        Robot_Stop();
        /* Discard damaged input and resynchronize at a fresh terminator.
           Never interpret a truncated command as a movement command. */
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        HAL_UART_AbortReceive(port);
        __HAL_UART_CLEAR_FLAG(port, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
        __HAL_UART_SEND_REQ(port, UART_RXDATA_FLUSH_REQUEST);
        rx_head = rx_tail = 0;
        rx_fault = 0;
        line_length = 0;
        discard_line = 1;
        if (HAL_UART_Receive_IT(port, &rx_byte, 1) != HAL_OK) rx_fault = 1;
        __set_PRIMASK(mask);
        reply("ERROR: UART RECEIVE ERROR/OVERFLOW; ROBOT STOPPED; SEND NEWLINE THEN RETRY\r\n");
    }
    /* Bound each visit so continuous serial traffic cannot starve safety. */
    for (unsigned budget = 0; budget < 32 && rx_tail != rx_head && !rx_fault; ++budget) {
        uint8_t ch = rx[rx_tail];
        rx_tail = (rx_tail + 1U) % RX_SIZE;
        if (ch == '\r' || ch == '\n') {
            if (!discard_line && line_length) {
                line[line_length] = '\0';
                Robot_Update();
                command();
                run_events(); /* Report each command's effect, even when lines arrive together. */
            }
            line_length = discard_line = 0;
        } else if (!discard_line) {
            if ((ch < 32 && ch != '\t') || ch > 126 || line_length >= LINE_SIZE - 1U) {
                discard_line = 1;
                line_length = 0;
                Robot_Stop();
                reply("ERROR: INVALID/OVERLONG LINE; ROBOT STOPPED; LINE DISCARDED\r\n");
            } else line[line_length++] = (char)ch;
        }
    }
    const RobotStatus *s = Robot_GetStatus();
    if (reported_turn != s->turn_number) {
        reported_turn = s->turn_number;
        reply("TURN: SW%u; POSITION=%lu; SERVO=%u US; NEXT=SW%u; DC=%d%%; HOME ERROR=%ld\r\n",
              s->turn_switch, (unsigned long)s->turn_position, s->turn_pulse_us,
              s->target, s->duty, (long)s->home_step_error);
    }
    run_events();
    transmit();
}
