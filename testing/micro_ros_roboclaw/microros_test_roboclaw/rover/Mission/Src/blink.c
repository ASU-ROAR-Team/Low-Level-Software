#include "../Inc/blink.h"
#include "../../ECAL/Inc/led_driver.h"

void Blink_Init(void) {
    LED_Init();
}

// Simple non-blocking delay logic for demonstration
void Blink_Update(RobotState_t current_state) {
    static uint32_t last_tick = 0;
    uint32_t interval = 0;

    switch (current_state) {
        case STATUS_IDLE:    interval = 1000; break; // 1 second
        case STATUS_RUNNING: interval = 100;  break; // 100 ms
        case STATUS_ERROR:   
            LED_SetState(1); // Force ON
            return; 
    }

    // Standard Non-blocking logic
    if (HAL_GetTick() - last_tick >= interval) {
        LED_Toggle();
        last_tick = HAL_GetTick();
    }
}
