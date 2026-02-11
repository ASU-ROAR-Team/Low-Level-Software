#include "../Inc/led_driver.h"

void LED_Init(void) {
    // Note: Clock enable is usually done in main.c (MX_GPIO_Init)
    // But you can ensure state here
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // Start OFF (for Active Low)
}

void LED_SetState(uint8_t state) {
    // Handle Active Low logic here (0 sets pin high/off, 1 sets pin low/on)
    if (state) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); // ON
    } else {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);   // OFF
    }
}

void LED_Toggle(void) {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}
