#ifndef STATUS_LED_H
#define STATUS_LED_H

/* StatusLed_Init()
 * ----------------
 * Clear activity flags and turn off PB9/RX and PF1/TX after GPIO setup.
 *
 * Returns: None.
 * Global variables modified: rx_on, tx_on, rx_received, tx_completed and LED
 * GPIO.
 * Errors: None.
 */
void StatusLed_Init(void);

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
void StatusLed_Update(void);

/* StatusLed_TxComplete()
 * ----------------------
 * Post a TX event from the UART callback; no GPIO access or delay.
 *
 * Returns: None.
 * Global variables modified: tx_completed.
 * Errors: None.
 */
void StatusLed_TxComplete(void);

/* StatusLed_RxReceived()
 * ----------------------
 * Post an RX event from the UART callback; no GPIO access or delay.
 *
 * Returns: None.
 * Global variables modified: rx_received.
 * Errors: None.
 */
void StatusLed_RxReceived(void);

#endif
