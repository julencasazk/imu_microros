#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float64.h>
#include <sensor_msgs/msg/imu.h>

#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/quaternion.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"


#include "bno055_regs.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
//#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) (void)(fn)

// PRIVATE VARS
// ========================================
const unsigned int timer_period = RCL_MS_TO_NS(10);
rcl_publisher_t imu_publisher;


uint8_t raw_gyro[6] = {0};
bno055_gyro_t angular_vel_data;
bno055_lin_accel_t linear_accel_data;
uint8_t raw_lin[6] = {0};
bno055_quaternion_t orientation_data;
uint8_t raw_quat[8] = {0};
// ========================================
// END PRIVATE VARS


/*
IMU ROS Message
========================================================
std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id

geometry_msgs/Quaternion orientation
        float64 x 0
        float64 y 0
        float64 z 0
        float64 w 1
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
        float64 x
        float64 y
        float64 z
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
        float64 x
        float64 y
        float64 z
float64[9] linear_acceleration_covariance # Row major x, y z
==========================================================
*/
sensor_msgs__msg__Imu imu_msg;

static uart_port_t uart_port = UART_NUM_0;


// CALLBACK FUNCTIONS
// -------------------------------

static void timer_cb(rcl_timer_t* timer, int64_t last_call_time)
{
	if (timer != NULL)
	{
		imu_msg.angular_velocity = (geometry_msgs__msg__Vector3){
			.x = angular_vel_data.x,
			.y = angular_vel_data.y,
			.z = angular_vel_data.z
		};

		imu_msg.linear_acceleration = (geometry_msgs__msg__Vector3){
			.x = linear_accel_data.x,
			.y = linear_accel_data.y,
			.z = linear_accel_data.z
		};
		
		imu_msg.orientation = (geometry_msgs__msg__Quaternion){
			.w = orientation_data.w,
			.x = orientation_data.x,
			.y = orientation_data.y,
			.z = orientation_data.z
		};
		
		// Unknown covariances
		imu_msg.orientation_covariance[0] = -1.0; 
		imu_msg.angular_velocity_covariance[0] = -1.0; 
		imu_msg.linear_acceleration_covariance[0] = -1.0; 
			
		RCCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
	}
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
	RCCHECK(rclc_node_init_default(&node, "imu_sens_esp32", "", &support));

	// create executor (do this before adding timers)
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

    // TIMERS
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, timer_period, timer_cb));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // PUBLISHERS (use the correct type for Imu)
    RCCHECK(rclc_publisher_init_best_effort(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_data"
    ));

	//----------------------------
	// END PUBLISHERS

	// SUBSCRIPTIONS
	// ----------------------------

	// ---------------------------------
	// END SUBSCRIPTIONS
	
	// ----------------------------------
	// END TIMERS

	// Initialize messages
	memset(&imu_msg, 0, sizeof(imu_msg));


	while(1){
		
		RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
		vTaskDelay(pdMS_TO_TICKS(5));
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&imu_publisher, &node));

	RCCHECK(rclc_executor_fini(&executor));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}


void imu_read_task(void * arg)
{
	// Setup I2C Master BUS
	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = 0,
		.scl_io_num = GPIO_NUM_22,
		.sda_io_num = GPIO_NUM_21,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	
	i2c_master_bus_handle_t bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

	// Setup Slave Device
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = BNO055_I2C_ADDR_PRIMARY,
		.scl_speed_hz = 100000, // Fast mode not recommended with internal pullups
	};

	i2c_master_dev_handle_t dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
	
	vTaskDelay(pdMS_TO_TICKS(100));

	// BNO055 starts in CONFIGMODE after a reset, needs to be set to a 
	// operating mode before reading. Also, switching to any operating mode
	// from CONFIGMODE takes 19ms.
	
	// Set BNO055 to IMU mode
	uint8_t command_buff[2] = {BNO055_OPR_MODE, BNO055_OPERATION_MODE_IMU}; // Accel + Gyro, Relative Orientation
	esp_err_t err = i2c_master_transmit(dev_handle, command_buff, 2, -1);
	if (err != ESP_OK)
	{
		ESP_LOGE("IMU_TASK", "Failed to set BNO055 to IMU mode. Error: %d", err);
	}
	vTaskDelay(pdMS_TO_TICKS(200));

	// For IMU Sensor ROS msg fields:
	//		Angular Velocity -> GYR_DATA_<axis>_LSB and GYR_DATA_<axis>_MSB
	//		Orientation Quaternion -> QUA_DATA_<dof>_LSB and QUA_DATA_<dof>_MSB
	//		Linear Acceleration -> LIA_DATA_<axis>_LSB and LIA_DATA_<axis>_MSB

	while (true)
	{

		// ANGULAR VELOCITY
		uint8_t reg = BNO055_GYR_DATA_X_LSB;
		
		err = i2c_master_transmit(dev_handle, &reg, 1, pdMS_TO_TICKS(1000));
		if (err != ESP_OK)
		{
			ESP_LOGE("IMU_TASK", "Failed to select angular velocity register address. Error: %d", err);
		}
		
		err = i2c_master_receive(dev_handle, raw_gyro, 6, pdMS_TO_TICKS(1000));
		if (err != ESP_OK)
		{
			ESP_LOGE("IMU_TASK", "Failed to select angular velocity register address. Error: %d", err);
		}

		int16_t raw_gyro_x = (int16_t)(raw_gyro[1] << 8 | raw_gyro[0]);
		int16_t raw_gyro_y = (int16_t)(raw_gyro[3] << 8 | raw_gyro[2]);
		int16_t raw_gyro_z = (int16_t)(raw_gyro[5] << 8 | raw_gyro[4]);

		angular_vel_data.x = (float)raw_gyro_x;
		angular_vel_data.y = (float)raw_gyro_y;
		angular_vel_data.z = (float)raw_gyro_z;
		
		// ORIENTATION
		reg = BNO055_QUA_DATA_W_LSB;
		err = i2c_master_transmit(dev_handle, &reg, 1, pdMS_TO_TICKS(1000));

		if (err != ESP_OK)
		{
			ESP_LOGE("IMU_TASK", "Failed to select angular velocity register address. Error: %d", err);
		}
		err = i2c_master_receive(dev_handle, raw_quat, 8, pdMS_TO_TICKS(1000));

		if (err != ESP_OK)
		{
			ESP_LOGE("IMU_TASK", "Failed to select angular velocity register address. Error: %d", err);
		}
		int16_t raw_quat_w = (int16_t)(raw_quat[1] << 8 | raw_quat[0]);
		int16_t raw_quat_x = (int16_t)(raw_quat[3] << 8 | raw_quat[2]);
		int16_t raw_quat_y = (int16_t)(raw_quat[5] << 8 | raw_quat[4]);
		int16_t raw_quat_z = (int16_t)(raw_quat[7] << 8 | raw_quat[6]);

		orientation_data.w = (float)raw_quat_w;
		orientation_data.x = (float)raw_quat_x;
		orientation_data.y = (float)raw_quat_y;
		orientation_data.z = (float)raw_quat_z;

		// LINEAR ACCELERATION
		reg = BNO055_LIA_DATA_X_LSB;
		err = i2c_master_transmit(dev_handle, &reg, 1, pdMS_TO_TICKS(1000));
		if (err != ESP_OK)
		{
			ESP_LOGE("IMU_TASK", "Failed to select angular velocity register address. Error: %d", err);
		}
		err = i2c_master_receive(dev_handle, raw_lin, 6, pdMS_TO_TICKS(1000));
		if (err != ESP_OK)
		{
			ESP_LOGE("IMU_TASK", "Failed to select angular velocity register address. Error: %d", err);
		}

		int16_t raw_lin_x = (int16_t)(raw_lin[1] << 8 | raw_lin[0]);
		int16_t raw_lin_y = (int16_t)(raw_lin[3] << 8 | raw_lin[2]);
		int16_t raw_lin_z = (int16_t)(raw_lin[5] << 8 | raw_lin[4]);

		linear_accel_data.x = (float)raw_lin_x;
		linear_accel_data.y = (float)raw_lin_y;
		linear_accel_data.z = (float)raw_lin_z;

		vTaskDelay(pdMS_TO_TICKS(10)); // Fastest sensor is 100Hz
	}
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
	
	xTaskCreate(imu_read_task,
			"imu_read_task",
			4096,
			NULL,
			5,
			NULL);
#else
#error micro-ROS transports misconfigured
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

}
