#include "microros.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mission_motion.h"
#include "motor_config.h"
#include "hx711.h"
#include "tim.h"

//#include "roboclaw.h"
int32_t message_flag=0;
int32_t weight=0;

/*General (1 time)*/
rcl_publisher_t loadcell_publisher;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
/*Roboclaw*/
rcl_subscription_t RoboClaw_loadcell_subscriber;
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray RoboClaw_loadcell_msg;
static std_msgs__msg__Int32 LoadCell_msg;
std_msgs__msg__Int32 msg1;
int32_t publish_flag=0;
// Define a structure to hold your shared data
typedef struct {
    int32_t angles[6];
    int32_t load_cell_command;
} MotorSystemData_t;

 MotorSystemData_t g_motor_data;

/*IMU*/

//rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;


TaskHandle_t microros_task_h;

void App_microROS_Task_Create()
{
	xTaskCreate(
		microros_task,
		"",
		1000,
		NULL,
		0,
		&microros_task_h
	);
}

void microros_task(void *argument)
{
	microros_init();
//	MX_TIM3_Init();
//	HX711_Init(&htim3);
//	  HX711_SetCalibration(8454830, 617000, 635873);
	for(;;)
	{

		if(publish_flag == 1){
		//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			if (rcl_publish(&loadcell_publisher, &LoadCell_msg, NULL) != RCL_RET_OK) {
					    		            printf("Error publishing IMU (line %d)\n", __LINE__);
					    		        }
			publish_flag = 0;
		}

		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10000000));
		 vTaskDelay(10);
	}
  /* USER CODE END StartDefaultTask */
}

//void led_subscription_callback(const void * msgin)
//{
//  // Cast the received message to the correct type
//  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
//
//  // Logic: 1 = LED ON, 0 = LED OFF
//  if (msg->data % 2 == 0) {
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//  } else {
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//  }
//}
//void led_subscription_callback(const void * msgin){
//	const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32 *) msgin;
//
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//}
void roboclaw_subscription_callback(const void * msgin)
{
    const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;

   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    // Ensure we actually received data
    if (msg->data.size >= 7) {
    	g_motor_data.angles[0]=msg->data.data[0];
    	g_motor_data.angles[1] = msg->data.data[1];
    	g_motor_data.angles[2] = msg->data.data[2];
    	g_motor_data.angles[3] = msg->data.data[3];
    	g_motor_data.angles[4] = msg->data.data[4];
    	g_motor_data.angles[5] = msg->data.data[5];
    	g_motor_data.load_cell_command= msg->data.data[6];
    }

    Mission_MoveSingleMotor(1, g_motor_data.angles[0]);
    //Mission_MoveSingleMotor(2, g_motor_data.angles[1]);
    Mission_MoveSingleMotor(3, g_motor_data.angles[2]);
   // Mission_MoveSingleMotor(4, g_motor_data.angles[3]);
//    if(g_motor_data.load_cell_command==1){
//    	weight = HX711_GetWeight()/1000;
//    	LoadCell_msg.data=weight;
//    	publish_flag=1;
//    }

//    if(weight>10){
//    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
//    }
//    else{
//    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
//    }

//
//
//    	Mission_MoveSingleMotor(1, g_motor_data.angles[0]);
//    	Mission_MoveSingleMotor(2, g_motor_data.angles[1]);
//
//
//
//    	message_flag=1;
//    	//Mission_MoveAllMotors(g_motor_data.angles);
//       // Mission_MoveToPosition(angle1,angle2);
//    }
}

void microros_init()
{

	  /* USER CODE BEGIN StartDefaultTask */
	Mission_ARM_Init();
	    rmw_uros_set_custom_transport(
	      true,
	      (void *) MICROROS_UART,
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
	    static int32_t memory_pool[7]; // Create a buffer for 6 integers
	    RoboClaw_loadcell_msg.data.data = memory_pool;
	    RoboClaw_loadcell_msg.data.size = 0;
	    RoboClaw_loadcell_msg.data.capacity = 7;
	    allocator = rcl_get_default_allocator();
	    //create init_options
	    rclc_support_init(&support, 0, NULL, &allocator);
	    rclc_node_init_default(&node, "stm32_led_node", "", &support);
	    rclc_subscription_init_default(
	              	     &RoboClaw_loadcell_subscriber,
	              	     &node,
	              	     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
	              	     "/micro_ros_roboclaw");
	    rclc_publisher_init_default(
	            &loadcell_publisher, &node,
	            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg,Int32 ),
	            "load_cell_topic");
//	    rclc_subscription_init_default(
//	    	              	     &subscriber,
//	    	              	     &node,
//	    	              	     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
//	    	              	     "/micro_ros_roboclaw");
	    rclc_executor_init(&executor, &support.context, 1, &allocator);

	    rclc_executor_add_subscription(&executor, &RoboClaw_loadcell_subscriber, &RoboClaw_loadcell_msg, &roboclaw_subscription_callback, ON_NEW_DATA);
	       // rclc_executor_add_subscription(&executor, &subscriber, &msg1, &led_subscription_callback, ON_NEW_DATA);

}
