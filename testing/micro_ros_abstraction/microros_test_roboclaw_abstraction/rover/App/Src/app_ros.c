/*
 * app_ros.c
 *
 *  Created on: Feb 9, 2026
 *      Author: bavly
 */

#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>
#include "app_ros.h"






static rcl_node_t node;
static rclc_executor_t executor;

void App_ROS_Create(void)
{
    xTaskCreate(App_ROS_Task,
                "ROS_Task",
                6000,
                NULL,
                3,
                NULL);
}

void App_ROS_Task(void *argument)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "rover_node", "", &support);
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


