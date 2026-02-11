#ifndef APP_TASKS_H
#define APP_TASKS_H

void App_Tasks_Create(void);

/* Task entry points */
void App_Motor_Task(void *argument);
void App_Sensor_Task(void *argument);

#endif
