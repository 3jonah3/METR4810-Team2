#include "switch.h"
#include "main.h"

uint8_t Switch_IsPressed(uint8_t switch_ID)
{
    uint16_t pin;

    switch (switch_ID)
    {
        case 1:
            pin = GPIO_PIN_14;  // SW1: PC14
            break;

        case 2:
            pin = GPIO_PIN_11;  // SW2: PC11
            break;

        case 3:
            pin = GPIO_PIN_10;  // SW3: PC10
            break;

        case 4:
            pin = GPIO_PIN_15;  // SW4: PC15
            break;

        default:
            return 0;
    }

    // Inputs use pull-ups; pressing connects the pin to GND.
    return HAL_GPIO_ReadPin(GPIOC, pin) == GPIO_PIN_RESET;
}

uint8_t Switch_AnyPressed(void)
{
    return Switch_IsPressed(1)
        || Switch_IsPressed(2)
        || Switch_IsPressed(3)
        || Switch_IsPressed(4);
}

/* ES/start switch: active high, with PC3 configured as an input/pull-down. */
uint8_t Switch_StartIsPressed(void)
{
    return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_SET;
}