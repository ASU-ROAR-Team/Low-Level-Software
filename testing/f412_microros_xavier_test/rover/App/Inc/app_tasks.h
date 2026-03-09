#ifndef APP_TASKS_H
#define APP_TASKS_H
// Inside app_tasks.h (or similar shared header)

#include "FreeRTOS.h"
#include "queue.h"

// 1. Define the struct for your queue data
typedef struct {
    int32_t angles[6];
} MotorCommand_t;

// 2. Tell the compiler this queue exists somewhere in the project
extern QueueHandle_t MotorQueue;
void App_Tasks_Create(void);


/* Task entry points */
void App_Button_Task(void *argument);
void App_LED_Task(void *argument);
void App_LoadCell_Task(void *argument);
void Arm_Control_Task(void *argument);
#endif
