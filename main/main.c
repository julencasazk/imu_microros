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
#include <std_msgs/msg/bool.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
//#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) (void)(fn)

#define SPEED_THRESHOLD 25.0f


bool interrupt_flag = false;

rcl_publisher_t throttle_publisher;
rcl_publisher_t brake_publisher;
rcl_publisher_t steer_publisher;
rcl_publisher_t reverse_publisher;

rcl_subscription_t throttle_subscriber;
rcl_subscription_t brake_subscriber;
rcl_subscription_t steer_subscriber;
rcl_subscription_t reverse_subscriber;

std_msgs__msg__Float32 throttle_msg;
std_msgs__msg__Float32 brake_msg;
std_msgs__msg__Float32 steer_msg;
std_msgs__msg__Bool reverse_msg;

static uart_port_t uart_port = UART_NUM_0;

void throttle_cb(const void * msgin)
{
//	const std_msgs__msg__Float32 * incoming_msg = (const std_msgs__msg__Float32 *)msgin;
//	throttle_msg.data = incoming_msg->data;
	RCSOFTCHECK(rcl_publish(&throttle_publisher, msgin, NULL));
}

void brake_cb(const void * msgin)
{
//	const std_msgs__msg__Float32 * incoming_msg = (const std_msgs__msg__Float32 *)msgin;
//	brake_msg.data = incoming_msg->data;
	RCSOFTCHECK(rcl_publish(&brake_publisher, msgin, NULL));
}

void steer_cb(const void * msgin)
{
//	const std_msgs__msg__Float32 * incoming_msg = (const std_msgs__msg__Float32 *)msgin;
//	steer_msg.data = incoming_msg->data;
	RCSOFTCHECK(rcl_publish(&steer_publisher, msgin, NULL));
}

void reverse_cb(const void * msgin)
{
//	const std_msgs__msg__Bool * incoming_msg = (const std_msgs__msg__Bool *)msgin;
//	reverse_msg.data = incoming_msg->data;
	RCSOFTCHECK(rcl_publish(&reverse_publisher, msgin, NULL));
}




void micro_ros_task(void * arg)
{
	// Set custom transport (In this case UART through USB-to-UART Bridge)
	rmw_uros_set_custom_transport(
		true,
		(void *) &uart_port,
		esp32_serial_open,
		esp32_serial_close,
		esp32_serial_write,
		esp32_serial_read
	);

	// Wait for the micro-ROS Agent on the host computer to initialize
	while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "platooning_control_esp32_micro_ros", "", &support));

	// PUBLISHERS
	// --------------------------------------
	(void)rclc_publisher_init_best_effort(
		&throttle_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"throttle_cmd"
	);

	(void)rclc_publisher_init_best_effort(
		&brake_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"brake_cmd"
	);

	(void)rclc_publisher_init_best_effort(
		&steer_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"steer_cmd"
	);
	
	(void)rclc_publisher_init_best_effort(
		&reverse_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
		"reverse_cmd"
	);
	//----------------------------
	// END PUBLISHERS

	// SUBSCRIPTIONS
	// ----------------------------

	RCCHECK(rclc_subscription_init_best_effort(
		&throttle_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"throttle_debug"
	));
	
	RCCHECK(rclc_subscription_init_best_effort(
		&brake_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"brake_debug"
	));

	RCCHECK(rclc_subscription_init_best_effort(
		&steer_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"steer_debug"
	));

	RCCHECK(rclc_subscription_init_best_effort(
		&reverse_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
		"reverse_debug"
	));

	// ---------------------------------
	// END SUBSCRIPTIONS

	// Initialize messages
	memset(&throttle_msg, 0, sizeof(throttle_msg));
	memset(&brake_msg, 0, sizeof(brake_msg));
	memset(&steer_msg, 0, sizeof(steer_msg));
	memset(&reverse_msg, 0, sizeof(reverse_msg));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&throttle_subscriber,
		&throttle_msg,
		throttle_cb,
		ON_NEW_DATA
	));
	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&brake_subscriber,
		&brake_msg,
		brake_cb,
		ON_NEW_DATA
	));
	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&steer_subscriber,
		&steer_msg,
		steer_cb,
		ON_NEW_DATA
	));
	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&reverse_subscriber,
		&reverse_msg,
		reverse_cb,
		ON_NEW_DATA
	));



	while(1){
		
		RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&throttle_publisher, &node));
	RCCHECK(rcl_publisher_fini(&brake_publisher, &node));
	RCCHECK(rcl_publisher_fini(&steer_publisher, &node));
	RCCHECK(rcl_publisher_fini(&reverse_publisher, &node));

	RCCHECK(rcl_subscription_fini(&throttle_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&brake_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&steer_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&reverse_subscriber, &node));

	RCCHECK(rclc_executor_fini(&executor));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}



void app_main(void)
{
	
	esp_log_level_set("*", ESP_LOG_NONE);

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)

    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
#else
#error micro-ROS transports misconfigured
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM


}
