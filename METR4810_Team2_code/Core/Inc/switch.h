#ifndef SWITCH_H
#define SWITCH_H

#include <stdint.h>

/* Switch_IsPressed()
 * ------------------
 * Read a raw active-low GPIOC switch without debounce.
 *
 * switch_ID: SW1..SW4, numbered 1..4.
 *
 * Returns: 1 when pressed, otherwise 0.
 * Global variables modified: None.
 * Errors: An invalid switch id returns 0.
 */
uint8_t Switch_IsPressed(uint8_t switch_ID);

/* Switch_AnyPressed()
 * -------------------
 * Read all four raw active-low switches, including PC11 and SW3.
 *
 * Returns: 1 if any input is pressed, otherwise 0.
 * Global variables modified: None.
 * Errors: None.
 */
uint8_t Switch_AnyPressed(void);

/* Switch_StartIsPressed()
 * -----------------------
 * Read the separate active-high PC3 input, not the PC11 run switch.
 *
 * Returns: 1 when PC3 is high, otherwise 0.
 * Global variables modified: None.
 * Errors: None.
 */
uint8_t Switch_StartIsPressed(void);

#endif
