#include "status_led.h"
#include "stm32g4xx_hal.h"

/* Existing PCB mapping from the original LED test: both active HIGH.
   GPIO output modes and clocks are already configured by the .ioc. */
#define RX_FLASH_MS 150U
#define TX_FLASH_MS 150U
static uint32_t rx_tick, tx_tick;
static uint8_t rx_on, tx_on;
static volatile uint8_t rx_received, tx_completed;

void StatusLed_Init(void)
{
    rx_on = rx_received = 0;
    tx_on = tx_completed = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
}

void StatusLed_TxComplete(void)
{
    tx_completed = 1;
}

void StatusLed_RxReceived(void)
{
    rx_received = 1;
}

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
