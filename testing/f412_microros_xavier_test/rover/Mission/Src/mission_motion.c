/*
 * mission_motion.c
 *
 *  Created on: April 27, 2025
 *      Author: Bavly
 */

#include "mission_motion.h"
#include "motor_config.h"
#include "roboclaw.h"
#include "usart.h"
#define ACTIVE_ROBOCLAWS 2
// Private variables for all 3 RoboClaws
#if ACTIVE_ROBOCLAWS >= 1
    static SERIAL_HandleTypeDef *hserial_uart1;
    static RoboClaw_HandleTypeDef hroboclaw1;
#endif
#if ACTIVE_ROBOCLAWS >= 2
    static SERIAL_HandleTypeDef *hserial_uart2;
    static RoboClaw_HandleTypeDef hroboclaw2;
#endif
#if ACTIVE_ROBOCLAWS >=3
    static SERIAL_HandleTypeDef *hserial_uart3;
    static RoboClaw_HandleTypeDef hroboclaw3;
#endif




// Target positions for all 6 motors
static int32_t target_positions[6] = {0, 0, 0, 0, 0, 0};

// Motion active flags for all 6 motors
static bool motor_active[6] = {false, false, false, false, false, false};

// Helper function to convert degrees to encoder ticks based on motor PPR
static int32_t angle_to_ticks(uint8_t motor, int32_t angle_deg) {
    int32_t ppr = 0;
    switch(motor) {
        case 1: ppr = MOTOR1_PPR; break;
        case 2: ppr = MOTOR2_PPR; break;
        case 3: ppr = MOTOR3_PPR; break;
        case 4: ppr = MOTOR4_PPR; break;
        case 5: ppr = MOTOR5_PPR; break;
        case 6: ppr = MOTOR6_PPR; break;
        default: return 0;
    }
    return (angle_deg * ppr) / 360;
}

static void get_motor_info(uint8_t motor, RoboClaw_HandleTypeDef **roboclaw, uint8_t *is_m1) {
    switch(motor) {

#if ACTIVE_ROBOCLAWS >= 1
    		case 1:
               *roboclaw = &hroboclaw1;
               *is_m1 = 1;
               break;
           case 2:
               *roboclaw = &hroboclaw1;
               *is_m1 = 0;
               break;
#endif
#if ACTIVE_ROBOCLAWS >=2
           case 3:
               *roboclaw = &hroboclaw2;
               *is_m1 = 1;
              break;
           case 4:
               *roboclaw = &hroboclaw2;
               *is_m1 = 0;
              break;
#endif
#if ACTIVE_ROBOCLAWS >=3

           case 5:
               *roboclaw = &hroboclaw3;
               *is_m1 = 1;
               break;
           case 6:
               *roboclaw = &hroboclaw3;
               *is_m1 = 0;
               break;
#endif
        default:
            *roboclaw = NULL;
            *is_m1 = 0;
            break;
    }
}

// Helper function to clamp position based on motor limits
static int32_t clamp_position(uint8_t motor, int32_t position) {
    switch(motor) {
        case 1:
            if (position < MOTOR1_MIN) return MOTOR1_MIN;
            if (position > MOTOR1_MAX) return MOTOR1_MAX;
            break;
        case 2:
            if (position < MOTOR2_MIN) return MOTOR2_MIN;
            if (position > MOTOR2_MAX) return MOTOR2_MAX;
            break;
        case 3:
            if (position < MOTOR3_MIN) return MOTOR3_MIN;
            if (position > MOTOR3_MAX) return MOTOR3_MAX;
            break;
        case 4:
            if (position < MOTOR4_MIN) return MOTOR4_MIN;
            if (position > MOTOR4_MAX) return MOTOR4_MAX;
            break;
        case 5:
            if (position < MOTOR5_MIN) return MOTOR5_MIN;
            if (position > MOTOR5_MAX) return MOTOR5_MAX;
            break;
        case 6:
            if (position < MOTOR6_MIN) return MOTOR6_MIN;
            if (position > MOTOR6_MAX) return MOTOR6_MAX;
            break;
    }
    return position;
}

// Public Functions
void Mission_ARM_Init(void) {
    // Initialize serial communication (assuming all on same UART)

#if ACTIVE_ROBOCLAWS >= 1
    // Configure RoboClaw 1
	 hserial_uart1 = serial_init(ROBOCLAW1_UART);  // Adjust UART handle as needed
    hroboclaw1.hserial = hserial_uart1;
    hroboclaw1.packetserial_address = ROBOCLAW1_ADDR;
    hroboclaw1.timeout = ROBOCLAW_TIMEOUT;
   // roboClaw_init(&hroboclaw1);
#endif
#if ACTIVE_ROBOCLAWS >= 2
    // Configure RoboClaw 2
    hserial_uart2 = serial_init(ROBOCLAW2_UART);
    hroboclaw2.hserial = hserial_uart2;
    hroboclaw2.packetserial_address = ROBOCLAW2_ADDR;
    hroboclaw2.timeout = ROBOCLAW_TIMEOUT;
    //roboClaw_init(&hroboclaw2);
#endif
#if ACTIVE_ROBOCLAWS >=3
    // Configure RoboClaw 3
    hserial_uart3 = serial_init(ROBOCLAW3_UART);
    hroboclaw3.hserial = hserial_uart3;
    hroboclaw3.packetserial_address = ROBOCLAW3_ADDR;
    hroboclaw3.timeout = ROBOCLAW_TIMEOUT;
   // roboClaw_init(&hroboclaw3);
#endif
    // Reset all encoders to start from zero
    Mission_ResetAllEncoders();
}
Mission_Status_t Mission_MoveSingleMotor(uint8_t motor, int32_t angle) {
    RoboClaw_HandleTypeDef *roboclaw;
    uint8_t is_m1;

    if (motor < 1 || motor > 6) {
        return MISSION_INVALID_MOTOR;
    }

    // 1. Convert the incoming integer angle to raw encoder ticks
    int32_t target_ticks = angle_to_ticks(motor, angle);

    // Get RoboClaw and channel info
    get_motor_info(motor, &roboclaw, &is_m1);

    // 2. Clamp the ticks to your safe limits
    target_positions[motor-1] = clamp_position(motor, target_ticks);

    // Send command to appropriate motor
    if (is_m1) {
        SpeedAccelDeccelPositionM1(
            roboclaw,
            ACCEL_DEFAULT, SPEED_DEFAULT, DECCEL_DEFAULT,
            target_positions[motor-1],
            1  // absolute position
        );
    } else {
        SpeedAccelDeccelPositionM2(
            roboclaw,
            ACCEL_DEFAULT, SPEED_DEFAULT, DECCEL_DEFAULT,
            target_positions[motor-1],
            1  // absolute position
        );
    }

    motor_active[motor-1] = true;
    return MISSION_OK;
}

Mission_Status_t Mission_MoveAllMotors(int32_t angles[6]) {
    Mission_Status_t status;

    // Move each motor individually
    for (uint8_t i = 0; i < 6; i++) {
        status = Mission_MoveSingleMotor(i+1, angles[i]);
        if (status != MISSION_OK) {
            return status;
        }
    }

    return MISSION_OK;
}

Mission_Status_t Mission_MoveMotorPair(uint8_t motor1, int32_t angle1,
                                       uint8_t motor2, int32_t angle2) {
    RoboClaw_HandleTypeDef *roboclaw1, *roboclaw2;
    uint8_t is_m1_1, is_m1_2;

    if (motor1 < 1 || motor1 > 6 || motor2 < 1 || motor2 > 6) {
        return MISSION_INVALID_MOTOR;
    }

    // 1. Convert integer angles to ticks
    int32_t ticks1 = angle_to_ticks(motor1, angle1);
    int32_t ticks2 = angle_to_ticks(motor2, angle2);

    // Get info for both motors
    get_motor_info(motor1, &roboclaw1, &is_m1_1);
    get_motor_info(motor2, &roboclaw2, &is_m1_2);

    // Check if they're on the same RoboClaw
    if (roboclaw1 != roboclaw2) {
        Mission_MoveSingleMotor(motor1, angle1);
        Mission_MoveSingleMotor(motor2, angle2);
    } else {
        // Clamp and store positions based on the calculated ticks
        target_positions[motor1-1] = clamp_position(motor1, ticks1);
        target_positions[motor2-1] = clamp_position(motor2, ticks2);

        SpeedAccelDeccelPositionM1M2(
            roboclaw1,
            ACCEL_DEFAULT, SPEED_DEFAULT, DECCEL_DEFAULT, target_positions[motor1-1],
            ACCEL_DEFAULT, SPEED_DEFAULT, DECCEL_DEFAULT, target_positions[motor2-1],
            1  // absolute position
        );

        motor_active[motor1-1] = true;
        motor_active[motor2-1] = true;
    }

    return MISSION_OK;
}


void Mission_ResetAllEncoders(void) {
#if ACTIVE_ROBOCLAWS >= 1
	ResetEncoders(&hroboclaw1);
#endif
#if ACTIVE_ROBOCLAWS >= 2
	 ResetEncoders(&hroboclaw2);
#endif
#if ACTIVE_ROBOCLAWS >=3
	 ResetEncoders(&hroboclaw3);
#endif


    // Reset target positions and active flags
    for (uint8_t i = 0; i < 6; i++) {
        target_positions[i] = 0;
        motor_active[i] = false;
    }
}
