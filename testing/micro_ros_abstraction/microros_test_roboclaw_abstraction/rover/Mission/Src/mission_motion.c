#include "roboclaw.h"
extern SERIAL_HandleTypeDef *hserial_uart2;
extern RoboClaw_HandleTypeDef hroboclaw_mc2;
static int32_t target_position1 = 0;
static int32_t target_position2 = 0;
static bool motion_active = false;

void Mission_MoveToPosition(int32_t position1,int32_t position2)
{
    if (position1 < 0) position1 = 0;
    if (position1 > 5290) position1 = 5290;

    target_position1 = position1;
    motion_active = true;

    SpeedAccelDeccelPositionM1(
        &hroboclaw_mc2,
        2000, 3000, 1000,
        target_position1,
        1
    );
    if (position2 < 0) position2 = 0;
        if (position2 > 5290) position2 = 5290;

        target_position2 = position2;
        motion_active = true;

        SpeedAccelDeccelPositionM2(
            &hroboclaw_mc2,
            2000, 3000, 1000,
            target_position2,
            1
        );
}

void Mission_Motion_Run(void)
{
    if (!motion_active) return;

    // Optional: check encoder feedback here
    // If reached → motion_active = false;
}
