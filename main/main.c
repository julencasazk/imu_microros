#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define SPEED_THRESHOLD 25.0f


bool interrupt_flag = false;

rcl_publisher_t throttle_publisher;
rcl_publisher_t brake_publisher;
rcl_subscription_t speed_subscriber;

std_msgs__msg__Float32 msg;

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * incoming_msg = (const std_msgs__msg__Float32 *)msgin;
	msg.data = incoming_msg->data;
}


void control_task(void *arg)
{
	while(1){
		std_msgs__msg__Float32 brake_msg;
		std_msgs__msg__Float32 throttle_msg;
		if (msg.data < SPEED_THRESHOLD) {
			brake_msg.data = 0.0f; 
			throttle_msg.data = 1.0f; 
		} else {
			brake_msg.data = 1.0f;
			throttle_msg.data = 0.0f;
		}
		RCSOFTCHECK(rcl_publish(&brake_publisher, &brake_msg, NULL));
		RCSOFTCHECK(rcl_publish(&throttle_publisher, &throttle_msg, NULL));
		usleep(100);
	}

	vTaskDelete(NULL);
}


void micro_ros_task(void * arg)
{

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "platooning_control_esp32_micro_ros", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_best_effort(
		&throttle_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"throttle_cmd"));

	RCCHECK(rclc_publisher_init_best_effort(
		&brake_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"brake_cmd"));

	// create subscriber
	RCCHECK(rclc_subscription_init_best_effort(
		&speed_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"speed_debug"));


	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&speed_subscriber,
		&msg,
		subscription_callback,
		ON_NEW_DATA));

	msg.data = 0.0f;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&throttle_publisher, &node));
	RCCHECK(rcl_publisher_fini(&brake_publisher, &node));
	RCCHECK(rcl_subscription_fini(&speed_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

static size_t uart_port = UART_NUM_0;

void app_main(void)
{
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
	rmw_uros_set_custom_transport(
		true,
		(void *) &uart_port,
		esp32_serial_open,
		esp32_serial_close,
		esp32_serial_write,
		esp32_serial_read
	);

#else
#error micro-ROS transports misconfigured
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);

	usleep(2000000); // Sleep for 2 seconds to let micro-ROS initialize
	xTaskCreate(control_task,
			"control_task",
			4096,
			NULL,
			10,
			NULL);
}
