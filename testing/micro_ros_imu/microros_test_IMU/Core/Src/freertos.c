/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include "IMU.h"

#include <std_msgs/msg/int32.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
extern IMU_INFO imu;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 1000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//Micro-ROS Builtin Functions
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
// This function runs when a message is received on the topic
void led_subscription_callback(const void * msgin)
{
  // Cast the received message to the correct type
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  // Logic: 1 = LED ON, 0 = LED OFF
  if (msg->data == 1) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */

    // micro-ROS configuration
    rmw_uros_set_custom_transport(
      true,
      (void *) &huart1,
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__);
    }

    // ---------------- micro-ROS Node ----------------
    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_node_t node;
    rcl_publisher_t imu_publisher;
    rclc_executor_t executor;

    rclc_support_init(&support, 0, NULL, &allocator);

    // Node name: "stm32_imu_node"
    rclc_node_init_default(&node, "stm32_imu_node", "", &support);

    // Initialize publisher for sensor_msgs/Imu on topic "/imu/data"
    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data");

    // Initialize executor (no subscriptions needed now)
    rclc_executor_init(&executor, &support.context, 0, &allocator);

    // ---------------- IMU setup ----------------
    //IMU_Setup(&hi2c1);

    sensor_msgs__msg__Imu imu_msg;
    sensor_msgs__msg__Imu__init(&imu_msg);
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
    for(;;)
    {
        // Read IMU data
        IMU_Operation(&imu);

        // Fill sensor_msgs/Imu
        imu_msg.header.stamp.sec = 0;       // Could add timestamp later
        imu_msg.header.stamp.nanosec = 0;
        imu_msg.header.frame_id.data = "imu_link";
        imu_msg.header.frame_id.size = strlen("imu_link");
        imu_msg.header.frame_id.capacity = strlen("imu_link") + 1;

        // Orientation from Euler angles → convert to quaternion (simplified)
        imu_msg.orientation.x = imu.Roll;
        imu_msg.orientation.y = imu.Pitch;
        imu_msg.orientation.z = imu.Yaw;
        imu_msg.orientation.w = 1.0;


        // Angular velocity (deg/sec)
        imu_msg.angular_velocity.x = imu.GyroX;
        imu_msg.angular_velocity.y = imu.GyroY;
        imu_msg.angular_velocity.z = imu.GyroZ;

        // Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = imu.linAccX;
        imu_msg.linear_acceleration.y = imu.linAccY;
        imu_msg.linear_acceleration.z = imu.linAccZ;

        // Publish IMU message
        rcl_ret_t ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
        if (ret != RCL_RET_OK) {
            printf("Error publishing IMU data\n");
        }

        // Spin executor (no callbacks now, safe)
//        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        osDelay(50);  // 20Hz publishing rate
    }

    /* USER CODE END StartDefaultTask */
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

