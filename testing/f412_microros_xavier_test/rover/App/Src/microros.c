#include "microros.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mission_motion.h"
#include "app_tasks.h"
#include "motor_config.h"
#include "hx711.h"
#include "tim.h"

MotorCommand_t new_cmd;
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
sensor_msgs__msg__JointState Motors_msg;
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

//void roboclaw_subscription_callback(const void * msgin)
//{
//    const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
//
//   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
//    // Ensure we actually received data
//    if (msg->data.size >= 7) {
//    	g_motor_data.angles[0]=msg->data.data[0];
//    	g_motor_data.angles[1] = msg->data.data[1];
//    	g_motor_data.angles[2] = msg->data.data[2];
//    	g_motor_data.angles[3] = msg->data.data[3];
//    	g_motor_data.angles[4] = msg->data.data[4];
//    	g_motor_data.angles[5] = msg->data.data[5];
//    	g_motor_data.load_cell_command= msg->data.data[6];
//    }
//
//    Mission_MoveSingleMotor(1, g_motor_data.angles[0]);
//    //Mission_MoveSingleMotor(2, g_motor_data.angles[1]);
//    Mission_MoveSingleMotor(3, g_motor_data.angles[2]);
//   // Mission_MoveSingleMotor(4, g_motor_data.angles[3]);
//
//}

//void roboclaw_subscription_callback(const void * msgin)
//{
//    const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
//
//    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
//
//    // Ensure we actually received enough data
//    if (msg->data.size >= 6) {
//        MotorCommand_t new_cmd;
//
//        // Copy data from the micro-ROS message into our RTOS struct
//        for(int i = 0; i < 6; i++) {
//            new_cmd.angles[i] = msg->data.data[i];
//        }
//
//        // Send the command to the queue.
//        // 0 means: if the queue is full, don't wait, just drop the old message and move on.
//        if (MotorQueue != NULL) {
//            xQueueSend(MotorQueue, &new_cmd, 0);
//        }
//    }
//}
void roboclaw_subscription_callback(const void * msgin)
{
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);

    if (msg->position.size >= 6) {
       //MotorCommand_t new_cmd;

        // Copy data from the micro-ROS message into our RTOS struct.
        // Note: JointState positions are doubles, so we cast to int32_t.
        for(int i = 0; i < 6; i++) {
            new_cmd.angles[i] = (int32_t)msg->position.data[i];
        }

        // Send the command to the queue.
        if (MotorQueue != NULL) {
            xQueueSend(MotorQueue, &new_cmd, 0);
        }
    }
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
	    // --- JOINTSTATE MEMORY ALLOCATION ---
	        // Allocate memory arrays for the sequences. Max expected size is 7.
	        static double position_memory[7];
	        static double velocity_memory[7];
	        static double effort_memory[7];
	        static rosidl_runtime_c__String name_memory[7];

	        // Assign buffers to the message sequences
	        Motors_msg.position.data = position_memory;
	        Motors_msg.position.capacity = 7;
	        Motors_msg.position.size = 0;

	        Motors_msg.velocity.data = velocity_memory;
	        Motors_msg.velocity.capacity = 7;
	        Motors_msg.velocity.size = 0;

	        Motors_msg.effort.data = effort_memory;
	        Motors_msg.effort.capacity = 7;
	        Motors_msg.effort.size = 0;

	        Motors_msg.name.data = name_memory;
	        Motors_msg.name.capacity = 7;
	        Motors_msg.name.size = 0;
	        // ------------------------------------

	        // Initialize the Subscriber for JointState
	        rclc_subscription_init_best_effort(
	            &RoboClaw_loadcell_subscriber,
	            &node,
	            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
	            "/fk_joint_states");

	        // Initialize the Publisher
	        rclc_publisher_init_default(
	            &loadcell_publisher, &node,
	            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	            "load_cell_topic");

	        rclc_executor_init(&executor, &support.context, 1, &allocator);

	        // Add subscription to executor using Motors_msg
	        rclc_executor_add_subscription(
	            &executor,
	            &RoboClaw_loadcell_subscriber,
	            &Motors_msg,
	            &roboclaw_subscription_callback,
	            ON_NEW_DATA);
}
