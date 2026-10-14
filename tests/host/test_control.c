#include "robot_control.h"
#include "serial.h"
#include "stepper.h"
#include "status_led.h"
#include "servo.h"
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
static void reset_with_button(unsigned down)
{
    memset(&fake_tim1, 0, sizeof fake_tim1); fake_tim1.ARR = 999;
    memset(&fake_tim4, 0, sizeof fake_tim4);
    memset(&fake_tim16, 0, sizeof fake_tim16);
    memset(motor.active, 0, sizeof motor.active); memset(step.active, 0, sizeof step.active);
    memset(servos.active, 0, sizeof servos.active);
    pressed = tick = output_length = transmit_pending = hold_transmit = 0;
    pressed = down ? GPIO_PIN_11 : 0;
    StatusLed_Init(); Robot_Init(&motor, &step, &servos); Serial_Init(&uart); pump();
}
static void reset(void) { reset_with_button(0); }
static void button(unsigned down)
{
    pressed = down ? GPIO_PIN_11 : 0;
    Robot_Update(); tick += 30; Robot_Update();
}
static void pulse(void)
{
    CHECK(fake_tim16.CR1 & TIM_CR1_OPM);
    CHECK(fake_tim16.CR1 & TIM_CR1_CEN);
    CHECK((fake_tim16.CCMR1 & TIM_CCMR1_OC1M) == TIM_OCMODE_PWM2);
    /* Model hardware overflow: OPM stops before invoking the callback. */
    CLEAR_BIT(fake_tim16.CR1, TIM_CR1_CEN);
    HAL_TIM_PeriodElapsedCallback(&step);
}

static void continuous_pulses(unsigned count)
{
    CHECK(Stepper1_IsBusy() && (fake_tim16.DIER & TIM_IT_UPDATE));
    CHECK(!(fake_tim16.CR1 & TIM_CR1_OPM));
    CHECK((fake_tim16.CCMR1 & TIM_CCMR1_OC1M) == TIM_OCMODE_PWM2);
    while (count--) {
        fake_tim16.CNT = 0;
        HAL_TIM_PeriodElapsedCallback(&step);
    }
    pump();
}

static void test_demo(void)
{
    reset();
    send("stepper(speed,333)\n");
    uint32_t period = fake_tim16.ARR;
    send("run\n");
    CHECK(Robot_GetStatus()->demo_homing && direction == 0 && !Robot_GetStatus()->duty);
    CHECK(fake_tim4.CCR[0] == 1500 && servos.active[0] && !servos.active[1]);
    continuous_pulses(1200);
    CHECK(Robot_GetStatus()->demo_position == 0);
    /* Home requires a stable press; bounce resumes homing. */
    pressed = GPIO_PIN_15; pump(); pressed = 0; tick += 30; pump();
    CHECK(Robot_GetStatus()->demo_homing && Stepper1_IsBusy());
    pressed = GPIO_PIN_15; pump(); tick += 30; pump();
    CHECK(!Robot_GetStatus()->demo_homing && direction == 1 && Robot_GetStatus()->duty == -100);
    CHECK(fake_tim1.CCR[0] == 1000 && fake_tim1.CCR[1] == 0);
    CHECK(fake_tim16.ARR == period && Robot_GetStatus()->speed == 333);
    pressed = 0;
    const uint32_t points[] = {59200,70000,80000,90000,100000,110000,120000,130000,131800,140000,145000};
    const unsigned widths[] = {1500,1505,1520,1544,1578,1621,1673,1734,1745,1800,1833};
    uint32_t previous = 0;
    for (unsigned i = 0; i < sizeof points / sizeof points[0]; ++i) {
        continuous_pulses(points[i] - previous);
        CHECK(Robot_GetStatus()->demo_position == points[i] && fake_tim4.CCR[0] == widths[i]);
        previous = points[i];
    }
    send("run\n"); CHECK(!Robot_GetStatus()->demo_homing && Robot_GetStatus()->demo_position == 145000);
    continuous_pulses(1000); CHECK(fake_tim4.CCR[0] == 1833);
    pressed = GPIO_PIN_14; pump(); tick += 30; pump(); pressed = 0;
    CHECK(direction == 0 && Robot_GetStatus()->duty == 100 && Robot_GetStatus()->demo_position == 146000);
    CHECK(fake_tim1.CCR[1] == 1000 && fake_tim1.CCR[0] == 0);
    continuous_pulses(1000); CHECK(Robot_GetStatus()->demo_position == 145000 && fake_tim4.CCR[0] == 1833);
    previous = 145000;
    for (int i = 9; i >= 0; --i) {
        continuous_pulses(previous - points[i]);
        CHECK(Robot_GetStatus()->demo_position == points[i] && fake_tim4.CCR[0] == widths[i]);
        previous = points[i];
    }
    continuous_pulses(59200); CHECK(Robot_GetStatus()->demo_position == 0 && fake_tim4.CCR[0] == 1500);
    pressed = GPIO_PIN_15; pump(); tick += 30; pump(); pressed = 0;
    CHECK(direction == 1 && Robot_GetStatus()->duty == -100 && Robot_GetStatus()->demo_position == 0);
    continuous_pulses(95000); CHECK(fake_tim4.CCR[0] == 1561); /* interpolation */
    send("status\n"); CHECK(strstr(output, "95000 STEPS") && strstr(output, "1561 US"));
    send("stop\n"); CHECK(!Stepper1_IsBusy() && !Robot_GetStatus()->duty && !servos.active[0]);
    send("stepper(speed,1000)\n"); send("run\n");
    CHECK(Robot_GetStatus()->demo_homing && fake_tim16.ARR == 999 && fake_tim4.CCR[0] == 1500);
    send("stop\n");
    /* Stop must count a pulse whose rising edge has already occurred. */
    Stepper1_Start(0); uint32_t before = Stepper1_TotalPulses();
    fake_tim16.CNT = fake_tim16.CCR[0]; fake_tim16.SR = TIM_FLAG_UPDATE;
    Stepper1_Stop(); CHECK(Stepper1_TotalPulses() == before + 2);
    Stepper1_Stop(); CHECK(Stepper1_TotalPulses() == before + 2);
    /* Independent stepper(auto) does not enable the demo or servos. */
    send("stepper(auto)\n"); continuous_pulses(80000);
    CHECK(Robot_GetStatus()->state == ROBOT_IDLE && !servos.active[0]);
    send("stop\n");
    /* A simultaneous-limit fault stops all outputs in the demo too. */
    send("run\n"); pressed = GPIO_PIN_14 | GPIO_PIN_15; pump();
    CHECK(Robot_GetStatus()->fault && !Stepper1_IsBusy() && !servos.active[0] && !Robot_GetStatus()->duty);
}

static void test_early_and_late_turns(void)
{
    static unsigned outward[160001];
    const unsigned endpoints[] = {100000U, 145000U, 160000U};
    for (unsigned n = 0; n < 3; ++n) {
        reset();
        CHECK(Robot_SetSpeed(n == 0 ? 333U : 2000U) == NULL);
        pressed = GPIO_PIN_15; send("run\n"); tick += 30; pump(); pressed = 0;
        CHECK(Robot_GetStatus()->duty == -100);
        outward[0] = fake_tim4.CCR[0];
        for (unsigned position = 1; position <= endpoints[n]; ++position) {
            continuous_pulses(1);
            outward[position] = fake_tim4.CCR[0];
        }
        /* A real early limit reverses at its actual coordinate, not 145000. */
        pressed = GPIO_PIN_14; pump(); tick += 30; pump(); pressed = 0;
        CHECK(Robot_GetStatus()->outward_steps == endpoints[n]);
        CHECK(Robot_GetStatus()->turn_position == endpoints[n]);
        CHECK(Robot_GetStatus()->turn_pulse_us == outward[endpoints[n]]);
        CHECK(Robot_GetStatus()->duty == 100 && direction == 0);
        CHECK(fake_tim4.CCR[0] == outward[endpoints[n]]);
        for (int position = (int)endpoints[n] - 1; position >= 0; --position) {
            continuous_pulses(1);
            CHECK(Robot_GetStatus()->demo_position == (uint32_t)position);
            CHECK(fake_tim4.CCR[0] == outward[position]);
            if (position == 75000) {
                /* A false home-switch trigger must not reset the path reference. */
                uint32_t turns = Robot_GetStatus()->turn_number;
                pressed = GPIO_PIN_15; pump(); pressed = 0; tick += 30; pump();
                CHECK(Robot_GetStatus()->turn_number == turns && Stepper1_IsBusy());
                CHECK(Robot_GetStatus()->demo_position == 75000 && fake_tim4.CCR[0] == outward[75000]);
            }
        }
        pressed = GPIO_PIN_15; pump(); tick += 30; pump();
        CHECK(Robot_GetStatus()->home_step_error == 0 && Robot_GetStatus()->duty == -100);
        CHECK(Robot_GetStatus()->return_steps == endpoints[n]);
        send("status\n"); CHECK(strstr(output, "LAST HOME ERROR=0 STEPS"));
    }
    reset(); pressed = GPIO_PIN_15; send("run\n"); tick += 30; pump(); pressed = 0;
    continuous_pulses(145000);
    pressed = GPIO_PIN_14; pump(); tick += 30; pump(); pressed = 0;
    continuous_pulses(144000);
    pressed = GPIO_PIN_15; pump(); tick += 30; pump();
    CHECK(Robot_GetStatus()->home_step_error == 1000); /* Stable early home is reported. */
    send("status\n"); CHECK(strstr(output, "LAST HOME ERROR=1000 STEPS"));
}

int main(void)
{
    test_early_and_late_turns();
    test_demo();
    reset();
    CHECK(strstr(output, "Welcome to METR4810") && strstr(output, "(O,O)") && strstr(output, "stepper(speed,") && strstr(output, "servo(angle)"));
    CHECK(strstr(output, "Welcome to METR4810 Team 2 CLI") < strstr(output, "(O,O)"));
    CHECK(strstr(output, "another accepted movement command"));
    CHECK(Robot_GetStatus()->state == ROBOT_IDLE);
    CHECK(!Stepper1_IsBusy() && !Robot_GetStatus()->duty && !servos.active[0] && !servos.active[1]);
    send("pInG\r\n"); CHECK(!strcmp(output, "OK PONG\r\n"));
    send("HELP\n"); CHECK(strstr(output, "available commands"));
    send("STATUS\r"); CHECK(strstr(output, "STATE: IDLE") && strstr(output, "PC11 RUN BUTTON: RELEASED"));
    /* All UART motion is available immediately after reset. */
    send("servo(90)\n"); CHECK(servos.active[0]); send("stop\n");
    send("motor(50)\n"); CHECK(Robot_GetStatus()->duty == 50); send("stop\n");
    send("stepper(1)\n"); CHECK(Stepper1_IsBusy()); send("stop\n");
    send("stepper(auto)\n"); CHECK(Stepper1_IsBusy()); send("stop\n");
    send("RUN\n");
    CHECK(Robot_GetStatus()->state == ROBOT_RUNNING && Robot_GetStatus()->demo_homing && Robot_GetStatus()->duty == 0);
    CHECK(direction == 0 && Stepper1_IsBusy());
    send("servo(90)\n"); CHECK(strstr(output, "send stop first"));
    send("motor(50)\n"); CHECK(Robot_GetStatus()->duty == 0);
    send("stepper(10)\n"); CHECK(Robot_GetStatus()->step_mode == ROBOT_STEP_AUTO);
    pressed = GPIO_PIN_15; pump(); CHECK(!Stepper1_IsBusy());
    tick += 29; pump(); CHECK(!Stepper1_IsBusy());
    tick++; pump(); CHECK(Stepper1_IsBusy() && direction == 1);
    pressed = 0; pump(); pressed = GPIO_PIN_14; pump(); pressed = 0;
    tick += 30; pump(); CHECK(Stepper1_IsBusy() && direction == 1); /* bounce resumes same way */
    pressed = GPIO_PIN_14; pump(); send("STOP\n"); tick += 100; pump();
    CHECK(Robot_GetStatus()->state == ROBOT_STOPPED && !Stepper1_IsBusy() && !Robot_GetStatus()->duty);
    pressed = 0; send("RUN\n"); CHECK(Stepper1_IsBusy());
    send("motor(stop)\n"); CHECK(!Stepper1_IsBusy() && Robot_GetStatus()->state == ROBOT_STOPPED);
    send("RUN\n"); send("stepper(stop)\n"); CHECK(!Robot_GetStatus()->duty);
    send("stepper(speed,2000)\n"); CHECK(fake_tim16.ARR == 499);
    send("stepper(1)\n"); CHECK(Stepper1_Remaining() == 1); pulse(); pump(); CHECK(!Stepper1_IsBusy());
    send("stepper(-200)\n"); CHECK(direction == 1);
    for (unsigned i = 0; i < 200; ++i) pulse();
    pump(); CHECK(!Stepper1_IsBusy() && !Stepper1_Remaining());
    send("stepper(200)\n"); pulse(); send("STOP\n"); CHECK(!Stepper1_Remaining());
    HAL_TIM_PeriodElapsedCallback(&step); CHECK(!Stepper1_IsBusy());
    pressed = GPIO_PIN_15; send("stepper(1)\n"); CHECK(strstr(output, "TARGET LIMIT"));
    send("stepper(-5)\n"); CHECK(Stepper1_IsBusy()); pressed = GPIO_PIN_14; pump(); CHECK(!Stepper1_IsBusy());
    pressed = 0; send("servo(90)\n"); CHECK(fake_tim4.CCR[0] == 1500 && servos.active[0] && !servos.active[1]);
    send("servo(45)\n"); CHECK(fake_tim4.CCR[0] == 1000);
    send("servo(0)\n"); CHECK(fake_tim4.CCR[0] == 500);
    send("servo(180)\n"); CHECK(fake_tim4.CCR[0] == 2500 && servos.active[0]);
    send("servo(60)\n"); CHECK(fake_tim4.CCR[0] == 1167);
    send("servo(500u)\n"); CHECK(fake_tim4.CCR[0] == 500);
    send("servo(2500u)\n"); CHECK(fake_tim4.CCR[0] == 2500);
    send(" SeRvO( 1501U )\r\n"); CHECK(fake_tim4.CCR[0] == 1501 && !servos.active[1]);
    send("servo(status)\n"); CHECK(strstr(output, "90 DEG (NOMINAL), 1501 US"));
    CHECK(Servo_SetPulseUs(1, 1234) == HAL_OK && fake_tim4.CCR[1] == 1234 && !servos.active[1]);
    CHECK(Servo_SetPulseUs(0, 1500) == HAL_ERROR);
    CHECK(Servo_SetPulseUs(2, 499) == HAL_ERROR && fake_tim4.CCR[0] == 1501);
    CHECK(Servo_SetPulseUs(2, 2501) == HAL_ERROR);
    CHECK(Robot_SetServoPulseUs(1, 1500) == NULL && servos.active[1]);
    send("stop\n"); send("run\n");
    send("servo(1500u)\n"); CHECK(strstr(output, "send stop first") && servos.active[0] && fake_tim4.CCR[0] == 1500);
    send("STOP\n"); CHECK(!servos.active[0] && !servos.active[1]);
    const char *invalid[] = {"servo(181)\n", "servo(-1)\n", "servo(499u)\n", "servo(2501u)\n", "servo(-1500u)\n", "servo(1500uu)\n", "servo(1500ux)\n", "servo(u)\n", "servo(999999999999999999u)\n", "servo(1500.5u)\n", "servo(200)\n","SERVO 0 90\n","SERVO 1\n","motor(-101)\n","motor(101)\n", "motor(50x)\n",
        "stepper(speed,-100)\n","stepper(speed,0)\n","stepper(speed,2001)\n","stepper(0)\n","stepper(-2147483648)\n",
        "stepper(999999999999999999)\n","RUN EXTRA\n","STOP EXTRA\n","UNKNOWN\n","A B C D E\n","motor(50\n","motor(50))\n","motor(50)junk\n", "motor(5 0)\n", "motor(,50)\n", "servo(1,90)\n", "stepper(speed,)\n", "stepper(speed,,500)\n", "stepper((1))\n", "MOTOR 50\n", "STEPPER 20\n"};
    for (unsigned i = 0; i < sizeof invalid / sizeof invalid[0]; ++i) {
        send(invalid[i]); CHECK(strstr(output, "ERROR:")); CHECK(!Stepper1_IsBusy() && !Robot_GetStatus()->duty && !servos.active[0] && !servos.active[1]);
    }
    send("motor(50)\n"); CHECK(fake_tim1.CCR[1] == 500);
    send(" MoToR( -50 )\n"); CHECK(Robot_GetStatus()->duty == -50 && fake_tim1.CCR[0] == 500 && fake_tim1.CCR[1] == 0);
    send("motor(status)\n"); CHECK(strstr(output, "-50%"));
    send("status\n"); CHECK(strstr(output, "DC MOTOR: -50%"));
    send("motor(-100)\n"); CHECK(fake_tim1.CCR[0] == 1000);
    send("motor(100)\n"); CHECK(fake_tim1.CCR[1] == 1000 && fake_tim1.CCR[0] == 0);
    send("stop\n");
    send("servo()\nmotor()\nstepper()\n"); CHECK(!Robot_GetStatus()->duty && !Stepper1_IsBusy() && !servos.active[0] && !servos.active[1]);
    send("servo(status)\n"); CHECK(strstr(output, "SERVO 2 TARGET:"));
    send("motor(50)\n");
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
    send("motor(50)\n"); CHECK(!Robot_GetStatus()->duty);
    send("servo(1500u)\n"); CHECK(strstr(output, "FAULT LATCHED") && !servos.active[0]);
    reset(); CHECK(!Robot_GetStatus()->fault);
    pressed = GPIO_PIN_15; send("RUN\n"); CHECK(!Stepper1_IsBusy()); tick += 30; pump(); CHECK(direction == 1 && Stepper1_IsBusy());
    /* Tick wrap while stopped for debounce. */
    pressed = GPIO_PIN_14; tick = UINT32_MAX - 10U; pump(); pressed = 0;
    tick = 20; pump(); CHECK(Stepper1_IsBusy() && direction == 1);
    /* Button works through Robot_Update alone, without serial polling. */
    reset();
    pressed = GPIO_PIN_11; Robot_Update(); tick += 10;
    pressed = 0; Robot_Update(); tick += 30; Robot_Update();
    CHECK(Robot_GetStatus()->state == ROBOT_IDLE); /* short bounce ignored */
    pressed = GPIO_PIN_11; Robot_Update(); tick += 29; Robot_Update();
    CHECK(Robot_GetStatus()->state == ROBOT_IDLE);
    tick++; Robot_Update();
    CHECK(Robot_GetStatus()->state == ROBOT_RUNNING && Robot_GetStatus()->demo_homing && Robot_GetStatus()->duty == 0);
    tick += 1000; Robot_Update(); CHECK(Robot_GetStatus()->state == ROBOT_RUNNING);
    pressed = 0; Robot_Update(); tick += 10;
    pressed = GPIO_PIN_11; Robot_Update(); tick += 30; Robot_Update();
    CHECK(Robot_GetStatus()->state == ROBOT_RUNNING); /* release bounce ignored */
    button(0); button(1);
    CHECK(Robot_GetStatus()->state == ROBOT_STOPPED && !Stepper1_IsBusy() && !Robot_GetStatus()->duty);
    button(0); send("run\n"); button(1);
    CHECK(Robot_GetStatus()->state == ROBOT_STOPPED); /* UART run, button stop */
    button(0); button(1); send("stop\n"); tick += 100; Robot_Update();
    CHECK(Robot_GetStatus()->state == ROBOT_STOPPED); /* held button cannot undo stop */
    button(0); button(1); CHECK(Robot_GetStatus()->state == ROBOT_RUNNING);
    pressed |= GPIO_PIN_14 | GPIO_PIN_15; Robot_Update();
    CHECK(Robot_GetStatus()->fault && !Stepper1_IsBusy());
    button(0); button(1); CHECK(!Robot_GetStatus()->duty && !Stepper1_IsBusy());
    reset_with_button(1);
    tick += 100; Robot_Update(); CHECK(Robot_GetStatus()->state == ROBOT_IDLE);
    button(0); button(1); CHECK(Robot_GetStatus()->state == ROBOT_RUNNING);
    button(0); Robot_Stop();
    tick = UINT32_MAX - 10U; pressed = GPIO_PIN_11; Robot_Update();
    tick = 19U; Robot_Update(); CHECK(Robot_GetStatus()->state == ROBOT_RUNNING);
    printf("PASS: %u checks (parser, safety, state machine, driver counting model)\n", checks);
    return 0;
}
