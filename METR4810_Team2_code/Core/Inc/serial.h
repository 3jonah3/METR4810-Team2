#ifndef SERIAL_H
#define SERIAL_H
#include "stm32g4xx_hal.h"
void Serial_Init(UART_HandleTypeDef *uart);
void Serial_Update(void);
#endif
