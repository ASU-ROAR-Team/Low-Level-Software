#include "../Inc/app_main.h"
#include "../../Mission/Inc/blink.h" // Call Layer 3
#include "../../Mission/Inc/IMU.h" // Call Layer 3
#include "../../../Core/Inc/main.h"
#include "../../../Core/Inc/i2c.h"
// Simulating a changing robot state
RobotState_t global_state = STATUS_IDLE;
IMU_INFO imu;

void App_Init(void) {
	//Blink_Init();
	IMU_Setup(&hi2c1);
}

void App_Run(void) {
    // 1. Simulate some logic to change state over time
    // For test: Change state every 5 seconds
	IMU_Operation(&imu);
//    if (HAL_GetTick() > 10000) {
//        global_state = STATUS_ERROR;
//    } else if (HAL_GetTick() > 5000) {
//        global_state = STATUS_RUNNING;
//    }
//
//    // 2. Run the Service
//    Blink_Update(global_state);
}
