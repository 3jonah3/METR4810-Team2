#ifndef STATUS_LED_H
#define STATUS_LED_H

/* PB9 flashes when a byte is received and PF1 when a transmission completes (150 ms each). */
void StatusLed_Init(void);
/* Turns the LEDs on and off. Call on every main-loop pass. */
void StatusLed_Update(void);
/* Called by UART callbacks; no delays or GPIO work in ISR. */
void StatusLed_TxComplete(void);
void StatusLed_RxReceived(void);

#endif
