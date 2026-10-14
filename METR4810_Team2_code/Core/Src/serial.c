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
#define TX_SIZE 4096U
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
    if (HAL_UART_Receive_IT(port, &rx_byte, 1) != HAL_OK) rx_fault = 1;
    reply("\r\nWelcome to METR4810 Team 2 CLI\r\n\r\n ,_,\r\n(O,O)\r\n(   )\r\n-\"-\"------\r\n\r\nREADY: IDLE; type help\r\n\r\n");
    help();
}

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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != port) return;
    StatusLed_TxComplete();
    tx_tail = (tx_tail + tx_active) % TX_SIZE;
    tx_active = 0;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == port) rx_fault = 1;
}

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

static void limits(void)
{
    reply("LIMITS: SW1=%u SW2/PC11=%u SW3=%u SW4=%u (1=PRESSED)\r\n",
          Switch_IsPressed(1), Switch_IsPressed(2), Switch_IsPressed(3), Switch_IsPressed(4));
}

static void step_status(void)
{
    const RobotStatus *s = Robot_GetStatus();
    const char *modes[] = {"STOPPED", "AUTO", "MANUAL"};
    reply("STEPPER: %s; TARGET: SW%u; SPEED: %lu STEPS/S; REMAINING: %lu\r\n",
          modes[s->step_mode], s->target, (unsigned long)s->speed,
          (unsigned long)Stepper1_Remaining());
}

static void status(void)
{
    const RobotStatus *s = Robot_GetStatus();
    const char *states[] = {"IDLE", "RUNNING", "STOPPED"};
    reply("STATE: %s\r\nDC MOTOR: %d%%\r\nFAULT: %s\r\nPC11 RUN BUTTON: %s\r\nTX RESPONSES DROPPED: %u\r\n",
          states[s->state], s->duty, s->fault ? "LIMIT FAULT LATCHED" : "NONE",
          Switch_IsPressed(2) ? "PRESSED" : "RELEASED", tx_dropped);
    step_status();
    limits();
    reply("DEMO: %s; POSITION FROM SW4: %lu STEPS; SERVO 2: %d US\r\n",
          s->state != ROBOT_RUNNING ? "INACTIVE" : s->demo_homing ? "HOMING SW4" : "FOLLOWING TABLE",
          (unsigned long)s->demo_position, s->servo_pulse_us[1]);
    reply("PATH: OUTWARD END=%lu; RETURN PULSES=%lu; LAST HOME ERROR=%ld STEPS\r\n",
          (unsigned long)s->outward_steps, (unsigned long)s->return_steps, (long)s->home_step_error);
}

static void result(const char *error, const char *success)
{
    reply(error ? "ERROR: %s\r\n" : "OK: %s\r\n", error ? error : success);
}

static void help(void)
{
    reply("%s", "available commands:\r\nType one command and press Enter. Commands ignore letter case.\r\nUse whole numbers for angles, duty, steps and speed.\r\n\r\nGENERAL\r\n  help       Show this full command guide.\r\n  ping       Test the serial link; replies OK PONG.\r\n  status     Show robot state, stepper mode/target/speed/remaining,\r\n             signed motor duty, limits, run button and faults.\r\n             Also reports dropped serial response fragments.\r\n  limits     Read SW1-SW4: 1=pressed, 0=released; SW2 is PC11.\r\n\r\n");
    reply("%s", "AUTONOMOUS CONTROL\r\n  run        Home to SW4 first (DC off, servo 2 at 1500 us).\r\n             Then push toward SW1 (DC -100%), return to SW4 (DC +100%).\r\n             Servo 2 follows the step-position table both ways.\r\n             Uses CLI-selected speed; repeats until stop or fault.\r\n  stop       Stop DC motor and stepper, cancel pending moves/auto,\r\n             disable servo PWM, and enter STOPPED. UART stays active.\r\n             This is a software stop, not a hardware emergency stop.\r\n\r\n");
    reply("%s", "SERVO (physical servo 2 only, PB6)\r\n  servo(angle)  Set target angle from 0 to 180 degrees.\r\n                Example: servo(90). Enables servo 2 PWM only.\r\n  servo(1500u) Set pulse width, 500..2500 us (servo 2 only).\r\n  servo(status) Show target degrees and us; -1=uncommanded.\r\n                Target is not measured position; stop disables PWM\r\n                but preserves this last target value.\r\n  servo()       Show this guide without moving.\r\n\r\n");
    reply("%s", "DC MOTOR\r\n  motor(duty)   Set duty from -100 to +100 percent. Positive is\r\n                forward, negative reverse, and zero stops the motor.\r\n                Examples: motor(30), motor(-30), motor(0).\r\n  motor(stop)   Stop DC motor. During run, stops the whole robot.\r\n                motor(0) has the same stopping behavior.\r\n  motor(status) Show signed duty; negative indicates reverse.\r\n  motor()       Show this guide without moving.\r\n\r\n");
    reply("%s", "STEPPER\r\n  stepper(steps)    Move a signed number of STEP pulses, then stop.\r\n                    Positive goes toward SW4; negative toward SW1.\r\n                    Range: -1000000..-1 or 1..1000000.\r\n                    Examples: stepper(20), stepper(-20).\r\n                    Stops early at its target limit. Stop any current\r\n                    stepper motion before starting a counted move.\r\n  stepper(speed,n)  Set nominal speed, 16..2000 steps/second.\r\n                    Example: stepper(speed,100). Must be stopped;\r\n                    setting speed does not start movement.\r\n");
    reply("%s", "  stepper(auto)     Travel continuously between SW1 and SW4.\r\n                    Replaces a counted move; leaves DC duty unchanged.\r\n  stepper(stop)     Stop stepper and cancel pending steps/auto.\r\n                    During run, stops the whole robot.\r\n  stepper(status)   Show mode, target switch, speed and steps remaining.\r\n  stepper()         Show this guide without moving.\r\n\r\n");
    reply("%s", "MOTION RULES\r\n  PC11/SW2: press to run, press again to stop (30 ms debounce).\r\n  Release between presses; held at reset requires release first.\r\n  No startup interlock; UART commands work immediately.\r\n  During run, send stop before individual movement/speed commands.\r\n  SW1 and SW4 pressed together latch a fault: correct cause and reset.\r\n  IDLE means no full autonomous program; individual motors may move.\r\n  After stop, another accepted movement command can resume motion.\r\n\r\n");
}

static char *trim(char *text)
{
    while (*text == ' ' || *text == '\t') ++text;
    char *end = text + strlen(text);
    while (end > text && (end[-1] == ' ' || end[-1] == '\t')) --end;
    *end = '\0';
    return text;
}

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
            strcmp(words[0], "STEPPER")) goto syntax_error;
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
            reply("SERVO 2 TARGET: %d DEG (NOMINAL), %d US (-1=UNCOMMANDED; stop DISABLES PWM)\r\n",
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
    transmit();
}
