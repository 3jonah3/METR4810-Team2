#ifndef SWITCH_H
#define SWITCH_H

#include <stdint.h>

// Returns 1 if pressed, 0 if released or ID is invalid.
uint8_t Switch_IsPressed(uint8_t switch_ID);
/* Any of the four active-low limit switches. */
uint8_t Switch_AnyPressed(void);

/* PC3 start switch: returns 1 when driven high (3.3 V). */
uint8_t Switch_StartIsPressed(void);

#endif
