#include "app_main.h"
#include "FreeRTOS.h"
#include "task.h"

#include "app_tasks.h"
#include "app_ros.h"

void App_Main_Init(void)
{
    App_Tasks_Create();
    App_ROS_Create();
}
