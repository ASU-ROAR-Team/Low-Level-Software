#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mission_motion.h"
#include "hx711.h"
QueueHandle_t MotorQueue;
void App_Tasks_Create(void)
{
//	defaultTaskHandle = osThreadNew(Arm_Control_Task, NULL, &defaultTask_attributes);
//    xTaskCreate(App_Button_Task,
//                "Button",
//                128,
//                NULL,
//                2,
//                NULL);

	MotorQueue = xQueueCreate(10, sizeof(MotorCommand_t));

	    if (MotorQueue == NULL) {
	        // Queue creation failed!
	    }
    xTaskCreate(Arm_Control_Task,
                "ARM",
                128,
                NULL,
                2,
                NULL);

//    xTaskCreate(App_LED2_Task,
//				"LED2",
//				128,
//				NULL,
//				2,
//				NULL);
}

/* ---------------- TASKS ---------------- */

void Arm_Control_Task(void *argument){
	Mission_ARM_Init();
	MotorCommand_t current_cmd;
	while(1)
	{
		if (xQueueReceive(MotorQueue, &current_cmd, portMAX_DELAY) == pdPASS) {
		            Mission_MoveSingleMotor(1, current_cmd.angles[0]);
		            Mission_MoveSingleMotor(2, current_cmd.angles[1]);
		            Mission_MoveSingleMotor(3, current_cmd.angles[2]);
		            Mission_MoveSingleMotor(4, current_cmd.angles[3]);

		        }
	}
}
void App_Button_Task(void *argument)
{
    while (1)
    {

    }
}

void App_LED_Task(void *argument)
{
    while (1)
    {

    }
}

void App_LED2_Task(void *argument)
{
    while (1)
    {

    }
}
void App_LoadCell_Task(void *argument){


	while(1)
	{

	}
}
