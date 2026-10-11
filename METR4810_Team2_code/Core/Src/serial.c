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
#define TX_SIZE 2048U
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
    if (HAL_UART_Receive_IT(port, &rx_byte, 1) != HAL_OK) rx_fault = 1;
    reply("READY: IDLE; TYPE HELP\r\n");
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
    reply("STATE: %s\r\nDC MOTOR: %u%%\r\nFAULT: %s\r\nPC11 STARTUP: %s\r\nTX RESPONSES DROPPED: %u\r\n",
          states[s->state], s->duty, s->fault ? "LIMIT FAULT LATCHED" : "NONE",
          s->unlocked ? "SATISFIED (LATCHED)" : "WAITING", tx_dropped);
    step_status();
    limits();
}

static void result(const char *error, const char *success)
{
    reply(error ? "ERROR: %s\r\n" : "OK: %s\r\n", error ? error : success);
}

static void command(void)
{
    char *words[4];
    unsigned count = 0;
    char *p = line;
    while (*p) {
        while (*p == ' ' || *p == '\t') ++p;
        if (!*p) break;
        if (count == 4) { reply("ERROR: TOO MANY PARAMETERS\r\n"); return; }
        words[count++] = p;
        while (*p && *p != ' ' && *p != '\t') { *p = (char)toupper((unsigned char)*p); ++p; }
        if (*p) *p++ = '\0';
    }
    if (!count) return;
    int32_t a, b;
    if (!strcmp(words[0], "PING") && count == 1) reply("OK PONG\r\n");
    else if (!strcmp(words[0], "HELP") && count == 1)
        reply("AVAILABLE COMMANDS:\r\nPING | HELP | STATUS | RUN | STOP | LIMITS\r\n"
              "STEPPER <signed steps> | SPEED <16..2000> | AUTO | STOP | STATUS\r\n"
              "SERVO <1|2> <0..120> | STATUS\r\nMOTOR <0..100> | STOP | STATUS\r\n"
              "RUN locks individual motion commands; send STOP first.\r\n");
    else if (!strcmp(words[0], "STATUS") && count == 1) status();
    else if (!strcmp(words[0], "LIMITS") && count == 1) limits();
    else if (!strcmp(words[0], "STOP") && count == 1) {
        Robot_Stop(); result(NULL, "ROBOT STOPPED\r\nDC MOTOR: OFF\r\nSTEPPER: STOPPED");
    } else if (!strcmp(words[0], "RUN") && count == 1)
        result(Robot_StartAuto(), "AUTONOMOUS MODE STARTED\r\nDC MOTOR: 100%\r\nSTEPPER: AUTO");
    else if (!strcmp(words[0], "STEPPER")) {
        if (count == 2 && !strcmp(words[1], "STATUS")) step_status();
        else if (count == 2 && !strcmp(words[1], "STOP")) {
            Robot_StopStepper(); result(NULL, "STEPPER STOPPED (RUN CANCELLED IF ACTIVE)");
        } else if (count == 2 && !strcmp(words[1], "AUTO")) result(Robot_StepperAuto(), "STEPPER AUTO");
        else if (count == 2 && number(words[1], &a)) result(Robot_Move(a), "STEPPER MOVE STARTED");
        else if (count == 3 && !strcmp(words[1], "SPEED") && number(words[2], &a))
            result(Robot_SetSpeed((uint32_t)a), "STEPPER SPEED SET");
        else reply("ERROR: USE STEPPER <steps>|SPEED <16..2000>|AUTO|STOP|STATUS\r\n");
    } else if (!strcmp(words[0], "MOTOR")) {
        if (count == 2 && !strcmp(words[1], "STATUS")) reply("DC MOTOR: %u%%\r\n", Robot_GetStatus()->duty);
        else if (count == 2 && !strcmp(words[1], "STOP")) {
            Robot_StopMotor(); result(NULL, "MOTOR STOPPED (RUN CANCELLED IF ACTIVE)");
        } else if (count == 2 && number(words[1], &a)) result(Robot_SetMotor((unsigned)a), "MOTOR DUTY SET");
        else reply("ERROR: USE MOTOR <0..100>|STOP|STATUS\r\n");
    } else if (!strcmp(words[0], "SERVO")) {
        if (count == 2 && !strcmp(words[1], "STATUS"))
            reply("SERVO TARGETS: 1=%d 2=%d DEG (-1=UNCOMMANDED; STOP DISABLES PWM)\r\n",
                  Robot_GetStatus()->servo_angle[0], Robot_GetStatus()->servo_angle[1]);
        else if (count == 3 && number(words[1], &a) && number(words[2], &b))
            result(Robot_SetServo((unsigned)a, (unsigned)b), "SERVO TARGET SET");
        else reply("ERROR: USE SERVO <1|2> <0..120>|STATUS\r\n");
    } else reply("ERROR: UNKNOWN COMMAND OR INVALID PARAMETERS\r\nTYPE HELP FOR AVAILABLE COMMANDS\r\n");
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
    transmit();
}
