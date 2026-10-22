#ifndef SERIAL_H
#define SERIAL_H
#include "stm32g4xx_hal.h"
/* UART command line. Starts interrupt reception and queues the welcome text and help. */
void Serial_Init(UART_HandleTypeDef *uart);
/* Run received commands, print run feedback and feed the transmitter. Call on every
   main-loop pass; it never waits for the UART. */
void Serial_Update(void);
#endif
