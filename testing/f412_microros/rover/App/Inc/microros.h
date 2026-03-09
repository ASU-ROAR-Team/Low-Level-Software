/*
 * microros.h
 *
 *  Created on: Feb 12, 2026
 *      Author: Khaled ElSisy
 */

#ifndef ROVER_APP_INC_MICROROS_H_
#define ROVER_APP_INC_MICROROS_H_

#include "usart.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include "mission_motion.h"
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>

#define MICROROS_UART &huart2

//Micro-ROS Builtin Functions
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

// User Function Prototypes
void led_subscription_callback(const void * msgin);
void microros_task(void *argument);
void microros_task();
void microros_init();
void App_microROS_Task_Create();

#endif /* ROVER_APP_INC_MICROROS_H_ */
