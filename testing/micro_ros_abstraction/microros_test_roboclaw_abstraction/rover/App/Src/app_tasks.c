#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"

#include "roboclaw.h"   // ECAL layer

void App_Tasks_Create(void)
{
    xTaskCreate(App_Motor_Task,
                "Motor_Task",
                2048,
                NULL,
                2,
                NULL);

    xTaskCreate(App_Sensor_Task,
                "Sensor_Task",
                2048,
                NULL,
                2,
                NULL);
}

/* ---------------- TASKS ---------------- */

void App_Motor_Task(void *argument)
{

    while (1)
    {
    	//roboclaw code
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void App_Sensor_Task(void *argument)
{
    while (1)
    {
        /* read sensors */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
