/*
 * mission_motion.h
 *
 *  Created on: April 27, 2025
 *      Author: Bavly
 */

#ifndef MISSION_MOTION_H_
#define MISSION_MOTION_H_

#include <stdint.h>
#include <stdbool.h>

// Simple status codes
typedef enum {
    MISSION_OK = 0,
    MISSION_BUSY,
    MISSION_ERROR,
    MISSION_INVALID_MOTOR
} Mission_Status_t;

// Initialize all motors
void Mission_ARM_Init(void);

// Move a single motor to position (motor = 1-6)
// Inside mission_motion.h

Mission_Status_t Mission_MoveSingleMotor(uint8_t motor, int32_t angle);
Mission_Status_t Mission_MoveAllMotors(int32_t angles[6]);
Mission_Status_t Mission_MoveMotorPair(uint8_t motor1, int32_t angle1, uint8_t motor2, int32_t angle2);
// Reset all encoders to zero
void Mission_ResetAllEncoders(void);

#endif /* MISSION_MOTION_H_ */
