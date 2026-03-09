#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mission_motion.h"
#include "hx711.h"
//typedef struct {
//	    int32_t angles[6];
//	    int32_t load_cell_command;
//	} MotorSystemData_t;
//
//	MotorSystemData_t g_motor_data;
void App_Tasks_Create(void)
{
//	defaultTaskHandle = osThreadNew(Arm_Control_Task, NULL, &defaultTask_attributes);
//    xTaskCreate(App_Button_Task,
//                "Button",
//                128,
//                NULL,
//                2,
//                NULL);

//    xTaskCreate(Arm_Control_Task,
//                "ARM",
//                128,
//                NULL,
//                2,
//                NULL);

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
	while(1)
	{

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
