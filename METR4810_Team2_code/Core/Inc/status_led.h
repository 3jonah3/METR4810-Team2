#ifndef STATUS_LED_H
#define STATUS_LED_H

void StatusLed_Init(void);
void StatusLed_Update(void);
/* Called by UART callbacks; no delays or GPIO work in ISR. */
void StatusLed_TxComplete(void);
void StatusLed_RxReceived(void);

#endif
