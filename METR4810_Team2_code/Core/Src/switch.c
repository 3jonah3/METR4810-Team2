/**
 * @file switch.c
 * Raw switch GPIO access. Active-low SW1..SW4 use pull-ups; PC3 is a separate
 * active-high input. Debouncing and run/limit policy belong to robot_control.
 */

#include "switch.h"
#include "main.h"

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
uint8_t Switch_IsPressed(uint8_t switch_ID)
{
    uint16_t pin;

    switch (switch_ID) {
    case 1:
        pin = GPIO_PIN_14; // SW1: PC14
        break;

    case 2:
        pin = GPIO_PIN_11; // SW2: PC11
        break;

    case 3:
        pin = GPIO_PIN_10; // SW3: PC10
        break;

    case 4:
        pin = GPIO_PIN_15; // SW4: PC15
        break;

    default:
        return 0;
    }

    // Inputs use pull-ups; pressing connects the pin to GND.
    return HAL_GPIO_ReadPin(GPIOC, pin) == GPIO_PIN_RESET;
}

/* Switch_AnyPressed()
 * -------------------
 * Read all four raw active-low switches, including PC11 and SW3.
 *
 * Returns: 1 if any input is pressed, otherwise 0.
 * Global variables modified: None.
 * Errors: None.
 */
uint8_t Switch_AnyPressed(void)
{
    return Switch_IsPressed(1)
            || Switch_IsPressed(2)
            || Switch_IsPressed(3)
            || Switch_IsPressed(4);
}

/* Switch_StartIsPressed()
 * -----------------------
 * Read the separate active-high PC3 input, not the PC11 run switch.
 *
 * Returns: 1 when PC3 is high, otherwise 0.
 * Global variables modified: None.
 * Errors: None.
 */
uint8_t Switch_StartIsPressed(void)
{
    return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_SET;
}