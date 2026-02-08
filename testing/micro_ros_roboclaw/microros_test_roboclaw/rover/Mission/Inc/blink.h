#ifndef BLINK_H
#define BLINK_H

#include <stdint.h>

// Define logical states for the robot
typedef enum
{
    STATUS_IDLE,    // Slow Blink
    STATUS_RUNNING, // Fast Blink
    STATUS_ERROR    // Solid ON
} RobotState_t;

void Blink_Init(void);
void Blink_Update(RobotState_t current_state);

#endif