#include "robot_control.h"
#include "serial.h"
#include "stepper.h"
#include "status_led.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

TIM_TypeDef fake_tim1, fake_tim4, fake_tim16;
static TIM_HandleTypeDef motor = {&fake_tim1}, step = {&fake_tim16}, servos = {&fake_tim4};
static UART_HandleTypeDef uart;
static uint32_t tick;
static unsigned pressed, direction, checks;
static uint8_t *receive;
static unsigned transmit_pending, hold_transmit;
static char output[32768];
static unsigned output_length;
#define CHECK(x) do { ++checks; if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
void Error_Handler(void) { CHECK(0); }
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
void HAL_NVIC_SetPriority(int i, int p, int s) { (void)i; (void)p; (void)s; }
void HAL_NVIC_EnableIRQ(int i) { (void)i; }
void HAL_NVIC_ClearPendingIRQ(int i) { (void)i; }
void HAL_GPIO_WritePin(int p, uint16_t pin, int v)
{ if (p == GPIOA && pin == GPIO_PIN_4) direction = v; }
int HAL_GPIO_ReadPin(int p, uint16_t pin) { (void)p; return !(pressed & pin); }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t, uint32_t c)
{ CHECK(!t->active[c]); t->active[c] = 1; SET_BIT(t->Instance->CR1, TIM_CR1_CEN); return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *t, uint32_t c)
{ t->active[c] = 0; if (!t->active[0] && !t->active[1]) CLEAR_BIT(t->Instance->CR1, TIM_CR1_CEN); return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *t, uint32_t e)
{ (void)e; t->Instance->CNT = 0; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *p, uint8_t *d, uint16_t n)
{ (void)p; (void)n; receive = d; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *p) { (void)p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *p, uint8_t *d, uint16_t n)
{
    (void)p;
    CHECK(output_length + n < sizeof output);
    memcpy(output + output_length, d, n);
    output_length += n; output[output_length] = 0;
    transmit_pending = 1;
    return HAL_OK;
}
static void pump(void)
{
    Robot_Update(); Serial_Update();
    StatusLed_Update();
    if (transmit_pending && !hold_transmit) { transmit_pending = 0; HAL_UART_TxCpltCallback(&uart); }
}
static void send(const char *text)
{
    output_length = 0; output[0] = 0;
    while (*text) { *receive = (uint8_t)*text++; HAL_UART_RxCpltCallback(&uart); pump(); }
    for (unsigned i = 0; i < 70; ++i) pump();
}
static void reset(void)
{
    memset(&fake_tim1, 0, sizeof fake_tim1); fake_tim1.ARR = 999;
    memset(&fake_tim4, 0, sizeof fake_tim4);
    memset(&fake_tim16, 0, sizeof fake_tim16);
    memset(motor.active, 0, sizeof motor.active); memset(step.active, 0, sizeof step.active);
    memset(servos.active, 0, sizeof servos.active);
    pressed = tick = output_length = transmit_pending = hold_transmit = 0;
    StatusLed_Init(); Robot_Init(&motor, &step, &servos); Serial_Init(&uart); pump();
}
static void unlock(void) { pressed = GPIO_PIN_11; pump(); pressed = 0; }
static void pulse(void)
{
    CHECK(fake_tim16.CR1 & TIM_CR1_OPM);
    CHECK(fake_tim16.CR1 & TIM_CR1_CEN);
    CHECK((fake_tim16.CCMR1 & TIM_CCMR1_OC1M) == TIM_OCMODE_PWM2);
    /* Model hardware overflow: OPM stops before invoking the callback. */
    CLEAR_BIT(fake_tim16.CR1, TIM_CR1_CEN);
    HAL_TIM_PeriodElapsedCallback(&step);
}

int main(void)
{
    reset();
    CHECK(Robot_GetStatus()->state == ROBOT_IDLE);
    CHECK(!Stepper1_IsBusy() && !Robot_GetStatus()->duty && !servos.active[0] && !servos.active[1]);
    send("pInG\r\n"); CHECK(!strcmp(output, "OK PONG\r\n"));
    send("HELP\n"); CHECK(strstr(output, "AVAILABLE COMMANDS"));
    send("STATUS\r"); CHECK(strstr(output, "STATE: IDLE") && strstr(output, "WAITING"));
    const char *blocked[] = {"RUN\n","STEPPER 1\n","STEPPER AUTO\n","MOTOR 50\n","SERVO 1 90\n"};
    for (unsigned i = 0; i < sizeof blocked / sizeof blocked[0]; ++i) {
        send(blocked[i]); CHECK(strstr(output, "INTERLOCK"));
    }
    unlock(); send("RUN\n");
    CHECK(Robot_GetStatus()->state == ROBOT_RUNNING && Robot_GetStatus()->duty == 100);
    CHECK(direction == 0 && Stepper1_IsBusy());
    send("SERVO 1 90\n"); CHECK(strstr(output, "SEND STOP FIRST"));
    send("MOTOR 50\n"); CHECK(Robot_GetStatus()->duty == 100);
    send("STEPPER 10\n"); CHECK(Robot_GetStatus()->step_mode == ROBOT_STEP_AUTO);
    pressed = GPIO_PIN_15; pump(); CHECK(!Stepper1_IsBusy());
    tick += 29; pump(); CHECK(!Stepper1_IsBusy());
    tick++; pump(); CHECK(Stepper1_IsBusy() && direction == 1);
    pressed = 0; pump(); pressed = GPIO_PIN_14; pump(); pressed = 0;
    tick += 30; pump(); CHECK(Stepper1_IsBusy() && direction == 1); /* bounce resumes same way */
    pressed = GPIO_PIN_14; pump(); send("STOP\n"); tick += 100; pump();
    CHECK(Robot_GetStatus()->state == ROBOT_STOPPED && !Stepper1_IsBusy() && !Robot_GetStatus()->duty);
    pressed = 0; send("RUN\n"); CHECK(Stepper1_IsBusy());
    send("MOTOR STOP\n"); CHECK(!Stepper1_IsBusy() && Robot_GetStatus()->state == ROBOT_STOPPED);
    send("RUN\n"); send("STEPPER STOP\n"); CHECK(!Robot_GetStatus()->duty);
    send("STEPPER SPEED 2000\n"); CHECK(fake_tim16.ARR == 499);
    send("STEPPER 1\n"); CHECK(Stepper1_Remaining() == 1); pulse(); pump(); CHECK(!Stepper1_IsBusy());
    send("STEPPER -200\n"); CHECK(direction == 1);
    for (unsigned i = 0; i < 200; ++i) pulse();
    pump(); CHECK(!Stepper1_IsBusy() && !Stepper1_Remaining());
    send("STEPPER 200\n"); pulse(); send("STOP\n"); CHECK(!Stepper1_Remaining());
    HAL_TIM_PeriodElapsedCallback(&step); CHECK(!Stepper1_IsBusy());
    pressed = GPIO_PIN_15; send("STEPPER 1\n"); CHECK(strstr(output, "TARGET LIMIT"));
    send("STEPPER -5\n"); CHECK(Stepper1_IsBusy()); pressed = GPIO_PIN_14; pump(); CHECK(!Stepper1_IsBusy());
    pressed = 0; send("SERVO 1 90\n"); CHECK(fake_tim4.CCR[1] == 1750 && servos.active[1]);
    send("SERVO 1 45\n"); CHECK(fake_tim4.CCR[1] == 1375);
    send("SERVO 2 120\n"); CHECK(fake_tim4.CCR[0] == 2000 && servos.active[0]);
    send("STOP\n"); CHECK(!servos.active[0] && !servos.active[1]);
    const char *invalid[] = {"SERVO 1 200\n","SERVO 0 90\n","SERVO 1\n","MOTOR -1\n","MOTOR 101\n", "MOTOR 50X\n",
        "STEPPER SPEED -100\n","STEPPER SPEED 0\n","STEPPER SPEED 2001\n","STEPPER 0\n","STEPPER -2147483648\n",
        "STEPPER 999999999999999999\n","RUN EXTRA\n","STOP EXTRA\n","UNKNOWN\n","A B C D E\n"};
    for (unsigned i = 0; i < sizeof invalid / sizeof invalid[0]; ++i) {
        send(invalid[i]); CHECK(strstr(output, "ERROR:")); CHECK(!Stepper1_IsBusy() && !Robot_GetStatus()->duty);
    }
    send("MOTOR 50\n"); CHECK(fake_tim1.CCR[1] == 500);
    char long_line[160]; memset(long_line, 'X', sizeof long_line); long_line[158] = '\n'; long_line[159] = 0;
    send(long_line); CHECK(strstr(output, "OVERLONG") && !Robot_GetStatus()->duty);
    send("PING\n"); CHECK(strstr(output, "PONG"));
    send("RUN\n");
    for (unsigned i = 0; i < 300; ++i) { *receive = 'X'; HAL_UART_RxCpltCallback(&uart); }
    pump(); CHECK(!Stepper1_IsBusy() && !Robot_GetStatus()->duty);
    send("RUN\n"); CHECK(!Stepper1_IsBusy()); /* resync discards first line */
    send("PING\n"); CHECK(strstr(output, "PONG"));
    send("RUN\n"); HAL_UART_ErrorCallback(&uart); pump(); CHECK(!Stepper1_IsBusy());
    send("\nPING\n"); CHECK(strstr(output, "PONG"));
    /* Backlogged TX must not prevent STOP being executed. */
    hold_transmit = 1;
    for (unsigned i = 0; i < 20; ++i) { send("HELP\n"); output_length = 0; }
    send("RUN\n"); CHECK(Stepper1_IsBusy()); send("STOP\n"); CHECK(!Stepper1_IsBusy());
    hold_transmit = 0; for (unsigned i = 0; i < 20; ++i) pump();
    pressed = GPIO_PIN_14 | GPIO_PIN_15; pump(); CHECK(Robot_GetStatus()->fault);
    pressed = 0; send("RUN\n"); CHECK(strstr(output, "FAULT LATCHED"));
    send("MOTOR 50\n"); CHECK(!Robot_GetStatus()->duty);
    reset(); CHECK(!Robot_GetStatus()->fault && !Robot_GetStatus()->unlocked);
    unlock(); pressed = GPIO_PIN_15; send("RUN\n"); CHECK(direction == 1);
    /* Tick wrap while stopped for debounce. */
    pressed = GPIO_PIN_14; tick = UINT32_MAX - 10U; pump(); pressed = 0;
    tick = 20; pump(); CHECK(Stepper1_IsBusy() && direction == 1);
    printf("PASS: %u checks (parser, safety, state machine, driver counting model)\n", checks);
    return 0;
}
