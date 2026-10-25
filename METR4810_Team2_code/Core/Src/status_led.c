/**
 * @file status_led.c
 * UART activity indicators. Interrupt callbacks set event flags; the main
 * loop consumes them and performs GPIO updates without delaying motion.
 */

#include "status_led.h"
#include "stm32g4xx_hal.h"

/* Existing PCB mapping from the original LED test: both active HIGH.
   GPIO output modes and clocks are already configured by the .ioc. */
#define RX_FLASH_MS 150U
#define TX_FLASH_MS 150U
static uint32_t rx_tick, tx_tick;
static uint8_t rx_on, tx_on;
static volatile uint8_t rx_received, tx_completed;

/* StatusLed_Init()
 * ----------------
 * Clear activity flags and turn off PB9/RX and PF1/TX after GPIO setup.
 *
 * Returns: None.
 * Global variables modified: rx_on, tx_on, rx_received, tx_completed and LED
 * GPIO.
 * Errors: None.
 */
void StatusLed_Init(void)
{
    rx_on = rx_received = 0;
    tx_on = tx_completed = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
}

/* StatusLed_TxComplete()
 * ----------------------
 * Post a TX event from the UART callback; no GPIO access or delay.
 *
 * Returns: None.
 * Global variables modified: tx_completed.
 * Errors: None.
 */
void StatusLed_TxComplete(void)
{
    tx_completed = 1;
}

/* StatusLed_RxReceived()
 * ----------------------
 * Post an RX event from the UART callback; no GPIO access or delay.
 *
 * Returns: None.
 * Global variables modified: rx_received.
 * Errors: None.
 */
void StatusLed_RxReceived(void)
{
    rx_received = 1;
}

/* StatusLed_Update()
 * ------------------
 * Consume UART event flags and maintain 150 ms LED flashes from main.
 * Briefly mask interrupts while consuming flags; never delay.
 *
 * Returns: None.
 * Global variables modified: rx_tick, tx_tick, rx_on, tx_on, event flags and
 * LED GPIO.
 * Errors: None.
 */
void StatusLed_Update(void)
{
    uint32_t now = HAL_GetTick();
    /* Atomically consume events so concurrent UART activity is not lost. */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint8_t completed = tx_completed;
    uint8_t received = rx_received;
    tx_completed = 0;
    rx_received = 0;
    __set_PRIMASK(mask);
    if (received) {
        rx_tick = now;
        rx_on = 1;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    } else if (rx_on && (uint32_t)(now - rx_tick) >= RX_FLASH_MS) {
        rx_on = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    }
    if (completed) {
        tx_tick = now;
        tx_on = 1;
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
    } else if (tx_on && (uint32_t)(now - tx_tick) >= TX_FLASH_MS) {
        tx_on = 0;
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
    }
}
