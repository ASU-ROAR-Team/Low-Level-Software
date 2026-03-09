#include "app_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_tasks.h"
#include "microros.h"

void App_Main_Init(void)
{

	App_microROS_Task_Create();
    App_Tasks_Create();
}
