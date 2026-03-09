#ifndef APP_TASKS_H
#define APP_TASKS_H

void App_Tasks_Create(void);

/* Task entry points */
void App_Button_Task(void *argument);
void App_LED_Task(void *argument);
void App_LoadCell_Task(void *argument);
void Arm_Control_Task(void *argument);
#endif
