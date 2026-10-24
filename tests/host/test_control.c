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
static unsigned servo_stops; /* HAL_TIM_PWM_Stop calls on the servo timer */
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
{ if (t == &servos) ++servo_stops; t->active[c] = 0; if (!t->active[0] && !t->active[1]) CLEAR_BIT(t->Instance->CR1, TIM_CR1_CEN); return HAL_OK; }
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
/* Power-up with the given switch pins held (GPIO_PIN_* mask). */
static void reset_pressed(unsigned pins)
{
    memset(&fake_tim1, 0, sizeof fake_tim1); fake_tim1.ARR = 999;
    memset(&fake_tim4, 0, sizeof fake_tim4);
    memset(&fake_tim16, 0, sizeof fake_tim16);
    memset(motor.active, 0, sizeof motor.active); memset(step.active, 0, sizeof step.active);
    memset(servos.active, 0, sizeof servos.active);
    pressed = tick = output_length = transmit_pending = hold_transmit = 0;
    pressed = pins;
    StatusLed_Init(); Robot_Init(&motor, &step, &servos); Serial_Init(&uart); pump();
}
static void reset_with_button(unsigned down) { reset_pressed(down ? GPIO_PIN_11 : 0); }
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

/* Start a run with the carriage off the switches and home it: SW4 first touch,
   debounce, then any overtravel. Leaves the run heading out toward SW1. */
static void run_and_home(void)
{
    pressed = 0; send("run\n");
    pressed = GPIO_PIN_15; pump(); tick += 30; pump();
#if LIMIT_OVERTRAVEL_STEPS
    for (uint32_t i = 0; i < LIMIT_OVERTRAVEL_STEPS; ++i) pulse();
    pump();
#endif
    pressed = 0;
}

/* Drive the run-start back-off off SW4; SW4 releases after release_after pulses. */
static void backoff_pulses(uint32_t release_after)
{
    CHECK(Robot_GetStatus()->demo_backoff && direction == 1 && Stepper1_Remaining() == RUN_START_BACKOFF_STEPS);
    for (uint32_t i = 0; i < RUN_START_BACKOFF_STEPS; ++i) {
        if (i == release_after) pressed &= ~GPIO_PIN_15;
        pulse();
    }
    pump();
}

#if LIMIT_OVERTRAVEL_STEPS == 0
static void test_demo(void)
{
    reset();
    send("stepper(speed,333)\n");
    uint32_t period = fake_tim16.ARR;
    unsigned stops = servo_stops;
    send("run\n");
    CHECK(Robot_GetStatus()->demo_homing && direction == 0 && !Robot_GetStatus()->duty);
    CHECK(fake_tim4.CCR[0] == 1556 && servos.active[0] && !servos.active[1]); /* 95 deg: start of the arc */
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
    /* CSV rows less the 1500-step CAD zero: positions count from SW4 first touch.
       158750 is SW1 first touch; the arc is capped at 135 degrees (2000 us). */
    const uint32_t points[] = {65215,68206,72195,76184,80172,84161,88150,92138,96127,100116,
        104104,108093,112082,116070,120059,124048,128037,132025,136014,140003,
        143353,143991,147980,151969,155957,158750};
    const unsigned widths[] = {1556,1559,1563,1570,1578,1588,1600,1613,1628,1644,
        1663,1683,1704,1727,1752,1779,1807,1836,1864,1893,
        1917,1922,1950,1979,2000,2000};
    uint32_t previous = 0;
    for (unsigned i = 0; i < sizeof points / sizeof points[0]; ++i) {
        continuous_pulses(points[i] - previous);
        CHECK(Robot_GetStatus()->demo_position == points[i] && fake_tim4.CCR[0] == widths[i]);
        CHECK(direction == 1 && Robot_GetStatus()->duty == -100);
        previous = points[i];
    }
    send("run\n"); CHECK(!Robot_GetStatus()->demo_homing && Robot_GetStatus()->demo_position == 158750);
    /* No step limit: only SW1 reverses; past the table the capped angle holds. */
    continuous_pulses(3000);
    CHECK(direction == 1 && Robot_GetStatus()->demo_position == 161750 && fake_tim4.CCR[0] == 2000);
    output_length = 0;
    pressed = GPIO_PIN_14; pump(); tick += 30; pump(); pressed = 0;
    CHECK(direction == 0 && Robot_GetStatus()->duty == 100 && Robot_GetStatus()->turn_switch == 1);
    CHECK(fake_tim1.CCR[1] == 1000 && fake_tim1.CCR[0] == 0);
    CHECK(strstr(output, "TURN: SW1; POSITION=161750; SERVO=2000 US; NEXT=SW4; DC=100%"));
    output_length = 0; tick += 1000; pump();
    CHECK(strstr(output, ",RETURN,161750,405.5,2000,22.2,100,0,0\r\n"));
    continuous_pulses(3000); CHECK(Robot_GetStatus()->demo_position == 158750 && fake_tim4.CCR[0] == 2000);
    previous = 158750;
    for (int i = 24; i >= 0; --i) {
        continuous_pulses(previous - points[i]);
        CHECK(Robot_GetStatus()->demo_position == points[i] && fake_tim4.CCR[0] == widths[i]);
        CHECK(direction == 0);
        previous = points[i];
    }
    continuous_pulses(65215); CHECK(Robot_GetStatus()->demo_position == 0 && fake_tim4.CCR[0] == 1556);
    /* One cycle per run: back at SW4 the run ends with the servo straight. */
    output_length = 0;
    uint32_t turns = Robot_GetStatus()->turn_number;
    pressed = GPIO_PIN_15; pump(); tick += 30; pump();
    CHECK(Robot_GetStatus()->state == ROBOT_STOPPED && Robot_GetStatus()->run_complete);
    CHECK(!Stepper1_IsBusy() && !Robot_GetStatus()->duty && fake_tim1.CCR[0] == 0 && fake_tim1.CCR[1] == 0);
    CHECK(servos.active[0] && fake_tim4.CCR[0] == 1556 && Robot_GetStatus()->demo_position == 0);
    CHECK(Robot_GetStatus()->home_step_error == 0 && Robot_GetStatus()->turn_number == turns);
    CHECK(strstr(output, "RUN COMPLETE: BACK AT SW4; HOME ERROR=0 STEPS; SERVO HOLDING 1556 US"));
    CHECK(!strstr(output, "TURN:"));
    for (unsigned i = 0; i < 20; ++i) { tick += 100; pump(); }
    CHECK(!Stepper1_IsBusy() && Robot_GetStatus()->state == ROBOT_STOPPED); /* does not repeat */
    send("status\n"); CHECK(strstr(output, "DEMO: COMPLETE"));
    /* The cycle ends on SW4, so the next run backs off 1500 steps, then homes normally. */
    send("run\n");
    CHECK(!Robot_GetStatus()->run_complete && Robot_GetStatus()->demo_homing && !Robot_GetStatus()->duty);
    CHECK(strstr(output, "RUN STARTED: SW4 PRESSED; BACKING OFF 1500 STEPS, THEN HOMING"));
    backoff_pulses(600);
    CHECK(!Robot_GetStatus()->demo_backoff && direction == 0 && Stepper1_IsBusy() && !(fake_tim16.CR1 & TIM_CR1_OPM));
    continuous_pulses(1500); CHECK(direction == 0 && Robot_GetStatus()->demo_homing);
    pressed = GPIO_PIN_15; pump(); tick += 30; pump(); pressed = 0;
    CHECK(direction == 1 && Robot_GetStatus()->duty == -100 && Robot_GetStatus()->demo_position == 0);
    continuous_pulses(95000); CHECK(fake_tim4.CCR[0] == 1624); /* interpolation */
    send("status\n"); CHECK(strstr(output, "95000 STEPS") && strstr(output, "1624 US"));
    output_length = 0; tick += 1000; pump();
    CHECK(strstr(output, ",OUT,95000,238.2,1624,3.4,-100,0,0\r\n"));
    /* stop freezes where it is: motion stops and the servo holds its angle. */
    send("stop\n");
    CHECK(strstr(output, "STOPPED AT: 95000 STEPS (238.2 MM FROM SW4) DURING OUT; SERVO HOLDING 1624 US"));
    CHECK(!Stepper1_IsBusy() && !Robot_GetStatus()->duty && servos.active[0] && fake_tim4.CCR[0] == 1624);
    send("servo(status)\n"); CHECK(strstr(output, "1624 US") && strstr(output, "stop HOLDS IT"));
    send("stepper(speed,1000)\n"); send("run\n"); /* re-homes without dropping the servo PWM */
    CHECK(Robot_GetStatus()->demo_homing && fake_tim16.ARR == 999 && fake_tim4.CCR[0] == 1556 && servos.active[0]);
    send("stop\n");
    CHECK(servo_stops == stops); /* run and stop never switched the servo PWM off */
    /* Stop must count a pulse whose rising edge has already occurred. */
    Stepper1_Start(0); uint32_t before = Stepper1_TotalPulses();
    fake_tim16.CNT = fake_tim16.CCR[0]; fake_tim16.SR = TIM_FLAG_UPDATE;
    Stepper1_Stop(); CHECK(Stepper1_TotalPulses() == before + 2);
    Stepper1_Stop(); CHECK(Stepper1_TotalPulses() == before + 2);
    /* Independent stepper(auto) does not drive the demo servo schedule. */
    reset(); run_and_home();
    continuous_pulses(150000); send("stop\n");
    unsigned held = fake_tim4.CCR[0];
    pressed = GPIO_PIN_15; send("stepper(auto)\n"); pressed = 0;
    CHECK(direction == 1);
    continuous_pulses(5000);
    CHECK(Robot_GetStatus()->state == ROBOT_IDLE && fake_tim4.CCR[0] == held && direction == 1 && Stepper1_IsBusy());
    send("stop\n");
    /* A simultaneous-limit fault stops motion; the servo holds its angle. */
    send("run\n"); pressed = GPIO_PIN_14 | GPIO_PIN_15; pump();
    CHECK(Robot_GetStatus()->fault && !Stepper1_IsBusy() && !Robot_GetStatus()->duty);
    CHECK(servos.active[0] && fake_tim4.CCR[0] == 1556);
}

static void test_turns(void)
{
    static unsigned outward[158751];
    /* Every outward leg reverses at SW1, wherever it is pressed; there is no step limit. */
    const unsigned endpoints[] = {100000U, 143353U, 158750U};
    for (unsigned n = 0; n < 3; ++n) {
        reset();
        CHECK(Robot_SetSpeed(n == 0 ? 333U : 2000U) == NULL);
        run_and_home();
        CHECK(Robot_GetStatus()->duty == -100);
        outward[0] = fake_tim4.CCR[0];
        for (unsigned position = 1; position <= endpoints[n]; ++position) {
            continuous_pulses(1);
            outward[position] = fake_tim4.CCR[0];
            CHECK(direction == 1);
        }
        output_length = 0;
        pressed = GPIO_PIN_14; pump(); tick += 30; pump(); pressed = 0;
        CHECK(strstr(output, "TURN: SW1; POSITION="));
        CHECK(Robot_GetStatus()->outward_steps == endpoints[n]);
        CHECK(Robot_GetStatus()->turn_position == endpoints[n]);
        CHECK(Robot_GetStatus()->turn_switch == 1);
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
        CHECK(Robot_GetStatus()->home_step_error == 0 && Robot_GetStatus()->run_complete);
        CHECK(Robot_GetStatus()->state == ROBOT_STOPPED && !Stepper1_IsBusy() && !Robot_GetStatus()->duty);
        CHECK(Robot_GetStatus()->return_steps == endpoints[n]);
        send("status\n"); CHECK(strstr(output, "LAST HOME ERROR=0 STEPS"));
    }
    CHECK(outward[158750] == 2000 && outward[143353] == 1917);
    reset(); run_and_home();
    continuous_pulses(140000);
    pressed = GPIO_PIN_14; pump(); tick += 30; pump(); pressed = 0;
    continuous_pulses(139000);
    pressed = GPIO_PIN_15; pump(); tick += 30; pump();
    CHECK(Robot_GetStatus()->home_step_error == 1000); /* Stable early home is reported. */
    send("status\n"); CHECK(strstr(output, "LAST HOME ERROR=1000 STEPS"));
    /* An early home still ends the run at zero with the servo straight. */
    CHECK(Robot_GetStatus()->demo_position == 0 && Robot_GetStatus()->run_complete && fake_tim4.CCR[0] == 1556);
    CHECK(!Stepper1_IsBusy() && Robot_GetStatus()->state == ROBOT_STOPPED);
    /* Mid-bore is not a turnaround. */
    run_and_home();
    continuous_pulses(143353);
    CHECK(direction == 1 && Robot_GetStatus()->demo_position == 143353 && fake_tim4.CCR[0] == 1917);
    /* A pulse in flight when SW1 stops the stepper is counted in the turn position. */
    reset(); run_and_home();
    continuous_pulses(120000);
    fake_tim16.CNT = fake_tim16.CCR[0]; pressed = GPIO_PIN_14; pump(); tick += 30; pump(); pressed = 0;
    CHECK(direction == 0 && Robot_GetStatus()->turn_switch == 1 && Robot_GetStatus()->outward_steps == 120001);
    /* Near SW1 the angle follows the table up to 135 degrees, then holds it. */
    reset(); run_and_home();
    continuous_pulses(153000); CHECK(direction == 1 && fake_tim4.CCR[0] == 1986);
    continuous_pulses(1788); CHECK(fake_tim4.CCR[0] == 1999);
    continuous_pulses(1); CHECK(fake_tim4.CCR[0] == 2000); /* 135 degrees */
    continuous_pulses(15211); CHECK(direction == 1 && fake_tim4.CCR[0] == 2000);
    send("stop\n"); CHECK(!Stepper1_IsBusy() && servos.active[0] && fake_tim4.CCR[0] == 2000);
}
#else
static void overtravel_pulses(void)
{
    CHECK(Stepper1_Remaining() == LIMIT_OVERTRAVEL_STEPS);
    for (uint32_t i = 0; i < LIMIT_OVERTRAVEL_STEPS; ++i) pulse();
    pump();
}

static void test_overtravel(void)
{
    const uint32_t ot = LIMIT_OVERTRAVEL_STEPS;
    reset();
    send("run\n");
    CHECK(Robot_GetStatus()->demo_homing && direction == 0 && fake_tim4.CCR[0] == 1556);
    continuous_pulses(5000);
    /* SW4 first touch: debounce, then a counted overtravel deeper into SW4, then zero. */
    pressed = GPIO_PIN_15; pump(); tick += 30; pump();
    CHECK(Robot_GetStatus()->demo_homing && direction == 0);
    output_length = 0;
    overtravel_pulses();
    CHECK(!Robot_GetStatus()->demo_homing && direction == 1 && Robot_GetStatus()->duty == -100);
    CHECK(Robot_GetStatus()->home_step_error == 0);
    CHECK(strstr(output, "TURN: SW4; POSITION=0; SERVO=1556 US; NEXT=SW1; DC=-100%; HOME ERROR=0"));
    CHECK(Robot_GetStatus()->demo_position == 0 && fake_tim4.CCR[0] == 1556);
    pressed = 0;
    /* With a 1500-step overtravel, positions are CSV steps directly. */
    continuous_pulses(95000); CHECK(fake_tim4.CCR[0] == 1618);
    continuous_pulses(160250 - 95000); CHECK(direction == 1 && fake_tim4.CCR[0] == 2000);
    /* SW1 first touch: the full overtravel, still at the capped angle, then reverse. */
    pressed = GPIO_PIN_14; pump(); tick += 30; pump();
    CHECK(direction == 1);
    overtravel_pulses(); pressed = 0;
    CHECK(direction == 0 && Robot_GetStatus()->turn_switch == 1 && fake_tim4.CCR[0] == 2000);
    CHECK(Robot_GetStatus()->outward_steps == 160250 + ot);
    /* SW4 first touch comes ot pulses before zero: zero home error, then overtravel. */
    continuous_pulses(160250);
    CHECK(Robot_GetStatus()->demo_position == ot);
    pressed = GPIO_PIN_15; pump(); tick += 30; pump();
    CHECK(Robot_GetStatus()->home_step_error == 0 && direction == 0);
    uint32_t turns = Robot_GetStatus()->turn_number;
    overtravel_pulses();
    /* After the SW4 overtravel the run ends at the zero point. */
    CHECK(Robot_GetStatus()->turn_number == turns && Robot_GetStatus()->run_complete);
    CHECK(Robot_GetStatus()->state == ROBOT_STOPPED && !Stepper1_IsBusy() && !Robot_GetStatus()->duty);
    CHECK(Robot_GetStatus()->demo_position == 0 && Robot_GetStatus()->return_steps == 160250 + ot);
    pressed = 0;
    /* An early SW1 also gets the overtravel and reverses from the actual position. */
    run_and_home();
    CHECK(direction == 1 && Robot_GetStatus()->duty == -100);
    continuous_pulses(100000);
    pressed = GPIO_PIN_14; pump(); tick += 30; pump();
    CHECK(direction == 1);
    overtravel_pulses(); pressed = 0;
    CHECK(direction == 0 && Robot_GetStatus()->turn_switch == 1);
    CHECK(Robot_GetStatus()->outward_steps == 100000 + ot);
    /* A run started on SW4 backs off 1500 + overtravel steps, then homes normally. */
    send("stop\n");
    pressed = GPIO_PIN_15; send("run\n");
    CHECK(Robot_GetStatus()->demo_homing && RUN_START_BACKOFF_STEPS == 1500U + ot);
    backoff_pulses(800);
    CHECK(direction == 0 && Stepper1_IsBusy() && Robot_GetStatus()->demo_homing && !Robot_GetStatus()->demo_backoff);
    CHECK(!(fake_tim16.CR1 & TIM_CR1_OPM));
    pressed = GPIO_PIN_15; pump(); tick += 30; pump();
    CHECK(direction == 0);
    overtravel_pulses();
    CHECK(!Robot_GetStatus()->demo_homing && direction == 1);
    /* SW4 still pressed 30 ms after the back-off latches a fault. */
    send("stop\n");
    send("run\n");
    backoff_pulses(UINT32_MAX);
    CHECK(Robot_GetStatus()->state == ROBOT_RUNNING && !Stepper1_IsBusy());
    tick += 29; pump(); CHECK(!Robot_GetStatus()->fault);
    tick += 1; pump();
    CHECK(Robot_GetStatus()->state == ROBOT_STOPPED && !Stepper1_IsBusy() && servos.active[0]);
    CHECK(Robot_GetStatus()->fault == ROBOT_FAULT_SW4_STUCK); /* latched: retries cannot creep toward SW1 */
    send("run\n"); CHECK(strstr(output, "FAULT LATCHED") && !Stepper1_IsBusy());
    /* Stop during an overtravel clears it; stepper(auto) then reacts to switches normally. */
    reset(); send("run\n"); pressed = GPIO_PIN_15; pump(); tick += 30; pump();
    CHECK(Stepper1_Remaining() == ot);
    send("stop\n"); CHECK(!Stepper1_IsBusy() && Robot_GetStatus()->state == ROBOT_STOPPED);
    pressed = 0; send("stepper(auto)\n");
    CHECK(direction == 0 && Stepper1_IsBusy() && !(fake_tim16.CR1 & TIM_CR1_OPM));
    pressed = GPIO_PIN_15; pump(); CHECK(!Stepper1_IsBusy());
    tick += 30; pump(); CHECK(direction == 0);
    overtravel_pulses();
    CHECK(direction == 1 && Stepper1_IsBusy() && Robot_GetStatus()->state == ROBOT_IDLE);
    pressed = 0;
    /* Re-issuing stepper(auto) mid-overtravel restarts cleanly and still sees the switches. */
    for (unsigned i = 0; i < 10; ++i) continuous_pulses(1);
    pressed = GPIO_PIN_14; pump(); tick += 30; pump(); CHECK(Stepper1_Remaining() == ot);
    for (unsigned i = 0; i < 100; ++i) pulse();
    send("stepper(auto)\n"); pressed = 0;
    CHECK(direction == 0 && Stepper1_IsBusy() && !(fake_tim16.CR1 & TIM_CR1_OPM));
    continuous_pulses(1000); pressed = GPIO_PIN_15; pump(); CHECK(!Stepper1_IsBusy());
    pressed = 0; send("stop\n");
}
#endif

/* Empty the capture buffer so negative strstr checks only see new output. */
static void clear_output(void) { output_length = 0; output[0] = 0; }

static void test_feedback(void)
{
    reset();
    tick += 5000; pump(); CHECK(!strstr(output, "DATA,")); /* nothing while idle */
    send("run\n");
    CHECK(strstr(output, "RUN STARTED: HOMING TO SW4") && strstr(output, "DATA,ms,leg,steps,mm,servo_us,pod_deg,dc,sw1,sw4\r\n"));
    char row[96]; snprintf(row, sizeof row, "DATA,%u,HOMING,0,0.0,1556,0.0,0,0,0\r\n", (unsigned)tick);
    CHECK(strstr(output, row));
    clear_output(); tick += 999; pump(); CHECK(!strstr(output, "DATA,"));
    tick += 1; pump(); CHECK(strstr(output, ",HOMING,"));
    send("telemetry(250)\n"); CHECK(strstr(output, "EVERY 250 MS"));
    clear_output(); tick += 249; pump(); CHECK(!strstr(output, "DATA,"));
    tick += 1; pump(); CHECK(strstr(output, "DATA,"));
    send("telemetry(off)\n"); CHECK(strstr(output, "OK: TELEMETRY OFF"));
    clear_output(); tick += 5000; pump(); CHECK(!strstr(output, "DATA,"));
    send("telemetry(on)\n"); /* header again when re-enabled mid-run */
    CHECK(strstr(output, "DATA,ms,leg") && strstr(output, "EVERY 250 MS"));
    send("stop\n"); CHECK(strstr(output, "STOPPED DURING HOMING (POSITION NOT SET); SERVO HOLDING 1556 US"));
    clear_output(); tick += 5000; pump(); CHECK(!strstr(output, "DATA,") && !strstr(output, "STOPPED"));
    /* A latched fault is reported once, with the stop. */
    send("run\n"); clear_output();
    pressed = GPIO_PIN_14 | GPIO_PIN_15; pump(); pressed = 0;
    CHECK(strstr(output, "FAULT: LIMIT FAULT LATCHED (SW1 AND SW4 PRESSED)") && strstr(output, "STOPPED DURING HOMING"));
    clear_output(); pump(); pump(); CHECK(!strstr(output, "FAULT:"));
    /* A run started on SW4 reports the back-off; a stuck SW4 latches its own fault. */
    reset(); pressed = GPIO_PIN_15; send("run\n");
    snprintf(row, sizeof row, "RUN STARTED: SW4 PRESSED; BACKING OFF %u STEPS, THEN HOMING",
             (unsigned)RUN_START_BACKOFF_STEPS);
    CHECK(strstr(output, row) && strstr(output, ",BACKOFF,0,0.0,1556,0.0,0,0,1\r\n"));
    send("status\n"); CHECK(strstr(output, "DEMO: BACKING OFF SW4"));
    backoff_pulses(UINT32_MAX); clear_output();
    tick += 30; pump();
    CHECK(Robot_GetStatus()->fault == ROBOT_FAULT_SW4_STUCK && !Stepper1_IsBusy());
    CHECK(strstr(output, "FAULT: LIMIT FAULT LATCHED (SW4 STILL PRESSED AFTER BACK-OFF)"));
    CHECK(strstr(output, "STOPPED DURING BACKOFF (POSITION NOT SET); SERVO HOLDING 1556 US"));
    send("status\n"); CHECK(strstr(output, "FAULT: LIMIT FAULT LATCHED (SW4 STUCK)"));
    pressed = 0;
    const char *bad[] = {"telemetry(99)\n", "telemetry(10001)\n", "telemetry(maybe)\n", "TELEMETRY ON\n",
        "telemetry(on,1)\n", "telemetry\n"};
    for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; ++i) { send(bad[i]); CHECK(strstr(output, "ERROR:")); }
    send("telemetry()\n"); CHECK(strstr(output, "TELEMETRY (printed during run)"));
    snprintf(row, sizeof row, "Starting on SW4: back off %u steps, then home.", (unsigned)RUN_START_BACKOFF_STEPS);
    send("help\n"); CHECK(strstr(output, row) && strstr(output, "(DC -100%)") && strstr(output, "1088u..2024u"));
    /* A fault already latched at power-up still prints its FAULT line. */
    reset_pressed(GPIO_PIN_14 | GPIO_PIN_15);
    CHECK(Robot_GetStatus()->fault && strstr(output, "FAULT: LIMIT FAULT LATCHED (SW1 AND SW4 PRESSED)"));
    pressed = 0;
    /* Starting servo 1 must not reset TIM4 while servo 2 is running (it would stretch PB6's pulse). */
    reset(); send("servo(1700u)\n"); CHECK(servos.active[0]);
    fake_tim4.CNT = 1234;
    CHECK(Robot_SetServoPulseUs(1, 1500) == NULL && servos.active[1] && fake_tim4.CNT == 1234);
    send("stop\n");
}

static void test_pause(void)
{
    char row[96];
    reset();
    send("pause\n"); CHECK(strstr(output, "ERROR: NO RUN TO PAUSE"));
    send("continue\n"); CHECK(strstr(output, "ERROR: NOT PAUSED"));
    run_and_home();
    CHECK(direction == 1 && Robot_GetStatus()->duty == -100);
    continuous_pulses(95000);
    unsigned servo_at = fake_tim4.CCR[0];
    /* pause freezes where it is but keeps the run */
    send("pause\n");
    CHECK(strstr(output, "OK: RUN PAUSED") && strstr(output, "PAUSED AT: 95000 STEPS (238.2 MM FROM SW4) DURING OUT"));
    CHECK(Robot_GetStatus()->state == ROBOT_RUNNING && Robot_GetStatus()->paused);
    CHECK(!Stepper1_IsBusy() && !Robot_GetStatus()->duty && fake_tim1.CCR[0] == 0 && fake_tim1.CCR[1] == 0);
    CHECK(servos.active[0] && fake_tim4.CCR[0] == servo_at);
    clear_output(); tick += 5000; pump(); CHECK(!strstr(output, "DATA,")); /* no rows while paused */
    send("status\n"); CHECK(strstr(output, "STATE: RUNNING (PAUSED)") && strstr(output, "DEMO: PAUSED"));
    /* while paused, run and manual motion are refused */
    send("run\n"); CHECK(strstr(output, "RUN PAUSED: send continue or stop"));
    send("motor(30)\n"); CHECK(strstr(output, "send stop first") && !Robot_GetStatus()->duty);
    send("stepper(100)\n"); CHECK(strstr(output, "send stop first") && !Stepper1_IsBusy());
    send("servo(1700u)\n"); CHECK(strstr(output, "send stop first") && fake_tim4.CCR[0] == servo_at);
    send("pause\n"); CHECK(strstr(output, "OK: RUN PAUSED") && !strstr(output, "PAUSED AT")); /* idempotent */
    /* continue resumes the same leg from the same position */
    send("continue\n");
    CHECK(strstr(output, "OK: RUN CONTINUING") && strstr(output, "CONTINUING: OUT") && strstr(output, ",OUT,95000,"));
    CHECK(!Robot_GetStatus()->paused && direction == 1 && Stepper1_IsBusy() && Robot_GetStatus()->duty == -100);
    CHECK(!(fake_tim16.CR1 & TIM_CR1_OPM));
    continuous_pulses(5000); CHECK(Robot_GetStatus()->demo_position == 100000);
    /* pause during a switch debounce: continue re-confirms the switch for a fresh 30 ms */
    pressed = GPIO_PIN_14; pump(); CHECK(!Stepper1_IsBusy());
    send("pause\n"); tick += 100; pump();
    CHECK(Robot_GetStatus()->target == 1 && Robot_GetStatus()->paused); /* no turn while paused */
    send("continue\n"); CHECK(!Stepper1_IsBusy() && Robot_GetStatus()->duty == -100);
    tick += 29; pump(); CHECK(Robot_GetStatus()->target == 1);
    tick += 1; pump();
#if LIMIT_OVERTRAVEL_STEPS
    /* the SW1 overtravel can itself be paused part-way and resumed */
    CHECK(Stepper1_Remaining() == LIMIT_OVERTRAVEL_STEPS);
    for (unsigned i = 0; i < 400; ++i) pulse();
    send("pause\n"); CHECK(!Stepper1_IsBusy());
    send("continue\n"); CHECK(Stepper1_Remaining() == LIMIT_OVERTRAVEL_STEPS - 400 && direction == 1);
    for (uint32_t i = 0; i < LIMIT_OVERTRAVEL_STEPS - 400; ++i) pulse();
    pump();
#endif
    pressed = 0;
    CHECK(Robot_GetStatus()->target == 4 && direction == 0 && Robot_GetStatus()->duty == 100); /* reversed at SW1 */
    /* stop while paused ends the run */
    send("pause\n"); send("stop\n");
    CHECK(Robot_GetStatus()->state == ROBOT_STOPPED && !Robot_GetStatus()->paused);
    CHECK(strstr(output, "STOPPED AT:") && strstr(output, "DURING RETURN"));
    send("continue\n"); CHECK(strstr(output, "ERROR: NOT PAUSED") && !Stepper1_IsBusy());
    /* pause during the run-start back-off resumes the remaining steps */
    pressed = GPIO_PIN_15; send("run\n");
    CHECK(Robot_GetStatus()->demo_backoff && Stepper1_Remaining() == RUN_START_BACKOFF_STEPS);
    for (unsigned i = 0; i < 500; ++i) pulse();
    send("pause\n"); CHECK(!Stepper1_IsBusy() && strstr(output, "PAUSED DURING BACKOFF (POSITION NOT SET)"));
    send("continue\n");
    CHECK(strstr(output, "CONTINUING: BACKOFF") && direction == 1 && Stepper1_Remaining() == RUN_START_BACKOFF_STEPS - 500);
    for (uint32_t i = 500; i < RUN_START_BACKOFF_STEPS; ++i) { if (i == 900) pressed = 0; pulse(); }
    pump(); CHECK(!Robot_GetStatus()->demo_backoff && direction == 0 && Robot_GetStatus()->demo_homing);
    /* the run button stops a paused run */
    send("pause\n"); CHECK(Robot_GetStatus()->paused);
    button(1); CHECK(Robot_GetStatus()->state == ROBOT_STOPPED && !Robot_GetStatus()->paused && !Stepper1_IsBusy());
    button(0);
    /* a fault while paused stops the run and latches */
    send("run\n"); send("pause\n"); CHECK(Robot_GetStatus()->paused);
    pressed = GPIO_PIN_14 | GPIO_PIN_15; pump(); pressed = 0;
    CHECK(Robot_GetStatus()->fault && Robot_GetStatus()->state == ROBOT_STOPPED && !Robot_GetStatus()->paused);
    send("continue\n"); CHECK(strstr(output, "FAULT LATCHED") && !Stepper1_IsBusy());
    /* A pulse already sent when pause lands is not repeated by continue. */
    reset(); pressed = GPIO_PIN_15; send("run\n");
    uint32_t first = Stepper1_TotalPulses();
    for (unsigned i = 0; i < 500; ++i) pulse();
    fake_tim16.CNT = fake_tim16.CCR[0]; /* pulse 501: rising edge out, update not yet taken */
    send("pause\n");
    CHECK(Robot_GetStatus()->pause_remaining == RUN_START_BACKOFF_STEPS - 501);
    send("stepper(status)\n");
    snprintf(row, sizeof row, "REMAINING: %u (PAUSED)", (unsigned)(RUN_START_BACKOFF_STEPS - 501));
    CHECK(strstr(output, row));
    send("continue\n");
    CHECK(Stepper1_Remaining() == RUN_START_BACKOFF_STEPS - 501 && !Robot_GetStatus()->pause_remaining);
    pressed = 0;
    for (uint32_t i = 501; i < RUN_START_BACKOFF_STEPS; ++i) pulse();
    pump();
    CHECK(Stepper1_TotalPulses() - first == RUN_START_BACKOFF_STEPS && !Robot_GetStatus()->demo_backoff);
    /* Lines arriving together still get one event line each. */
    send("pause\n"); clear_output();
    const char *both = "continue\npause\n";
    for (const char *c = both; *c; ++c) { *receive = (uint8_t)*c; HAL_UART_RxCpltCallback(&uart); }
    pump();
    CHECK(strstr(output, "CONTINUING: HOMING") && strstr(output, "PAUSED DURING HOMING"));
    send("stop\n");
}

static void test_arc(void)
{
    reset(); /* both servos wait at the 95 degree centre with their PWM off */
    CHECK(fake_tim4.CCR[0] == 1556 && fake_tim4.CCR[1] == 1556 && !servos.active[0] && !servos.active[1]);
    send("servo(95)\n"); CHECK(fake_tim4.CCR[0] == 1556 && servos.active[0]);
    send("servo(arc)\n");
    CHECK(strstr(output, "OK: SERVO ARC TEST 95 -> 135 -> 95 DEG (1556 -> 2000 US), ABOUT 9 S"));
    CHECK(Robot_GetStatus()->arc_test && servos.active[0] && fake_tim4.CCR[0] == 1556);
    CHECK(!Stepper1_IsBusy() && !Robot_GetStatus()->duty); /* nothing else moves */
    tick += 2000; pump(); CHECK(fake_tim4.CCR[0] == 1778);
    tick += 2000; pump(); CHECK(fake_tim4.CCR[0] == 2000); /* 135 degrees */
    tick += 999; pump(); CHECK(fake_tim4.CCR[0] == 2000);
    tick += 1; pump(); CHECK(fake_tim4.CCR[0] == 2000);
    tick += 2000; pump(); CHECK(fake_tim4.CCR[0] == 1778);
    send("servo(status)\n"); CHECK(strstr(output, "ARC TEST RUNNING"));
    clear_output(); tick += 1999; pump(); CHECK(Robot_GetStatus()->arc_test && fake_tim4.CCR[0] == 1556);
    tick += 1; pump();
    CHECK(!Robot_GetStatus()->arc_test && Robot_GetStatus()->arc_tests_done == 1);
    CHECK(servos.active[0] && fake_tim4.CCR[0] == 1556 && !Stepper1_IsBusy() && !Robot_GetStatus()->duty);
    CHECK(strstr(output, "SERVO ARC TEST DONE: SERVO HOLDING 1556 US"));
    clear_output(); tick += 5000; pump(); CHECK(!strstr(output, "ARC") && fake_tim4.CCR[0] == 1556);
    /* stop aborts the sweep and the servo holds where it is */
    send("servo(arc)\n"); tick += 1000; pump(); CHECK(fake_tim4.CCR[0] == 1667);
    send("stop\n");
    CHECK(!Robot_GetStatus()->arc_test && strstr(output, "SERVO ARC TEST STOPPED: SERVO HOLDING 1667 US"));
    tick += 3000; pump(); CHECK(fake_tim4.CCR[0] == 1667 && servos.active[0]);
    /* a manual servo command ends the sweep */
    send("servo(arc)\n"); tick += 500; pump();
    send("servo(1700u)\n");
    CHECK(!Robot_GetStatus()->arc_test && fake_tim4.CCR[0] == 1700 && strstr(output, "SERVO ARC TEST STOPPED"));
    tick += 3000; pump(); CHECK(fake_tim4.CCR[0] == 1700);
    /* run takes over: the sweep ends and the run homes with the servo at the centre */
    send("servo(arc)\n"); tick += 1500; pump();
    send("run\n"); CHECK(!Robot_GetStatus()->arc_test && Robot_GetStatus()->demo_homing && fake_tim4.CCR[0] == 1556);
    /* refused during a run and while a fault is latched */
    send("servo(arc)\n"); CHECK(strstr(output, "send stop first") && !Robot_GetStatus()->arc_test);
    send("stop\n");
    send("servo(arc)\n"); pressed = GPIO_PIN_14 | GPIO_PIN_15; pump(); pressed = 0;
    CHECK(Robot_GetStatus()->fault && !Robot_GetStatus()->arc_test);
    send("servo(arc)\n"); CHECK(strstr(output, "FAULT LATCHED") && !Robot_GetStatus()->arc_test);
    send("servo(arcs)\n"); CHECK(strstr(output, "ERROR:"));
    /* the guide shows the centre and the sweep */
    reset(); send("servo()\n");
    CHECK(strstr(output, "Example: servo(95), the centre.") && strstr(output, "servo(1556u) Set pulse width"));
    CHECK(strstr(output, "servo(arc)    Sweep the run's arc: 95 -> 135 -> 95 deg in about"));
}

int main(void)
{
#if LIMIT_OVERTRAVEL_STEPS
    test_overtravel();
#else
    test_turns();
    test_demo();
#endif
    test_feedback();
    test_pause();
    test_arc();
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
    tick++; pump();
#if LIMIT_OVERTRAVEL_STEPS
    CHECK(direction == 0 && Stepper1_Remaining() == LIMIT_OVERTRAVEL_STEPS); /* overtravel first */
    for (uint32_t i = 0; i < LIMIT_OVERTRAVEL_STEPS; ++i) pulse();
    pump();
#endif
    CHECK(Stepper1_IsBusy() && direction == 1);
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
    send("servo(1500u)\n"); CHECK(strstr(output, "send stop first") && servos.active[0] && fake_tim4.CCR[0] == 1556);
    send("STOP\n"); CHECK(servos.active[0] && servos.active[1] && fake_tim4.CCR[0] == 1556); /* held */
    reset();
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
    send("servo(1700u)\n"); CHECK(strstr(output, "FAULT LATCHED") && fake_tim4.CCR[0] != 1700);
    reset(); CHECK(!Robot_GetStatus()->fault);
    /* Started on SW4: back off, then home; the first-touch debounce spans a tick wrap. */
    pressed = GPIO_PIN_15; send("RUN\n");
    backoff_pulses(400);
    CHECK(direction == 0 && Stepper1_IsBusy() && Robot_GetStatus()->demo_homing);
    tick = UINT32_MAX - 10U; pressed = GPIO_PIN_15; pump(); CHECK(!Stepper1_IsBusy());
    tick = 18U; pump(); CHECK(!Stepper1_IsBusy()); /* 29 ms */
    tick = 19U; pump();
#if LIMIT_OVERTRAVEL_STEPS
    CHECK(direction == 0 && Stepper1_Remaining() == LIMIT_OVERTRAVEL_STEPS);
    for (uint32_t i = 0; i < LIMIT_OVERTRAVEL_STEPS; ++i) pulse();
    pump();
#endif
    pressed = 0;
    CHECK(Stepper1_IsBusy() && direction == 1 && !Robot_GetStatus()->demo_homing);
    /* Tick wrap while stopped for an SW1 debounce. */
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
    printf("PASS: %u checks (parser, safety, state machine, driver counting model; overtravel %u)\n",
           checks, (unsigned)LIMIT_OVERTRAVEL_STEPS);
    return 0;
}
