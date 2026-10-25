#ifndef SERIAL_H
#define SERIAL_H
#include "stm32g4xx_hal.h"

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
void Serial_Init(UART_HandleTypeDef* uart);

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
void Serial_Update(void);
#endif
