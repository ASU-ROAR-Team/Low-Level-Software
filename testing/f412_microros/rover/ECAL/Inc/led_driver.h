#ifndef ECAL_LED_DRIVER_H
#define ECAL_LED_DRIVER_H

#include "stm32f4xx_hal.h" // Change to f4xx if using F4 later

// Hardware Configuration (Decoupled from logic)
#define LED_PORT GPIOC
#define LED_PIN  GPIO_PIN_13

void LED_Init(void);
void LED_SetState(uint8_t state); // 1 = ON, 0 = OFF
void LED_Toggle(void);

#endif
