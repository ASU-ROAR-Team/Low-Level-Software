/*
 * motor_config.h
 *
 *  Created on: April 27, 2025
 *      Author: Bavly
 */

#ifndef MOTOR_CONFIG_H_
#define MOTOR_CONFIG_H_

#include "roboclaw.h"

// RoboClaw Configuration
#define ROBOCLAW1_ADDR    0x82
#define ROBOCLAW1_UART	  &huart1
#define ROBOCLAW2_ADDR    0x80
#define ROBOCLAW2_UART	  &huart6
#define ROBOCLAW3_ADDR    0x84
#define ROBOCLAW3_UART	  &huart6
#define ROBOCLAW_TIMEOUT  200

// Motion Parameters (easily adjustable)
#define ACCEL_DEFAULT     2000
#define SPEED_DEFAULT     3000
#define DECCEL_DEFAULT    1000

// Position Limits for each motor (if needed)
#define MOTOR1_MIN        0
#define MOTOR1_MAX        5290
#define MOTOR2_MIN        0
#define MOTOR2_MAX        5290
#define MOTOR3_MIN        0
#define MOTOR3_MAX        5290
#define MOTOR4_MIN        0
#define MOTOR4_MAX        5290
#define MOTOR5_MIN        0
#define MOTOR5_MAX        5290
#define MOTOR6_MIN        0
#define MOTOR6_MAX        5290

// Motor PPR (Pulses Per Revolution)
#define MOTOR1_PPR        5290
#define MOTOR2_PPR        5200
#define MOTOR3_PPR        5290
#define MOTOR4_PPR        5290
#define MOTOR5_PPR        3200
#define MOTOR6_PPR        3200

#endif /* MOTOR_CONFIG_H_ */
