#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/i2c.h"
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
#include "esp_timer.h"

#include "rosidl_runtime_c/string_functions.h"
#include "bno055_regs.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
//#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) (void)(fn)

// PRIVATE VARS
// ========================================
const unsigned int timer_period = RCL_MS_TO_NS(10);
rcl_publisher_t imu_publisher;

static const char IMU_FRAME_ID[] = "imu_link"; 

uint8_t raw_gyro[6] = {0};
bno055_gyro_t angular_vel_data;
bno055_lin_accel_t linear_accel_data;
uint8_t raw_lin[6] = {0};
bno055_quaternion_t orientation_data;
uint8_t raw_quat[8] = {0};
// ========================================
// END PRIVATE VARS

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

        // Stamp using ESP-IDF high-res timer (monotonic since boot)
        int64_t us = esp_timer_get_time();
        imu_msg.header.stamp.sec = (int32_t)(us / 1000000LL);
        imu_msg.header.stamp.nanosec = (uint32_t)((us % 1000000LL) * 1000);

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

    // Initialize message and set fixed frame_id
    sensor_msgs__msg__Imu__init(&imu_msg);
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, IMU_FRAME_ID);

    // Unknown covariances: follow ROS convention ([0] = -1, rest 0)
    imu_msg.orientation_covariance[0] = -1.0;
    imu_msg.angular_velocity_covariance[0] = -1.0;
    imu_msg.linear_acceleration_covariance[0] = -1.0;

    while(1){
        RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
    sensor_msgs__msg__Imu__fini(&imu_msg);

    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rcl_node_fini(&node));

      vTaskDelete(NULL);
}


void i2c_write(uint8_t reg, uint8_t *buff, uint8_t len, uint32_t timeout)
{
	i2c_cmd_handle_t handle = NULL;
	handle = i2c_cmd_link_create();
	if (handle != NULL)
	{
		i2c_master_start(handle);
		i2c_master_write_byte(handle, (BNO055_I2C_ADDR_PRIMARY << 1) | I2C_MASTER_WRITE, true); // Last bit from address is read/write
		i2c_master_write_byte(handle, reg, true);
		i2c_master_write(handle, (uint8_t*)&buff[0], len, true);
		i2c_master_stop(handle);
		
		esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, handle, pdMS_TO_TICKS(timeout));
		if (ret != ESP_OK)
		{
            ESP_LOGE("IMU_TASK", "Write failed: %s", esp_err_to_name(ret));
		}
		i2c_cmd_link_delete(handle);
	}
}


void i2c_read(uint8_t reg, uint8_t *buff, uint8_t len, uint32_t timeout)
{
	i2c_cmd_handle_t handle = NULL;
	handle = i2c_cmd_link_create();
	if (handle != NULL)
	{
		i2c_master_start(handle);
		i2c_master_write_byte(handle, (BNO055_I2C_ADDR_PRIMARY << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(handle, reg, true);
		i2c_master_start(handle);
		i2c_master_write_byte(handle, (BNO055_I2C_ADDR_PRIMARY << 1) | I2C_MASTER_READ, true);

		if (len > 0) // Must not be empty
		{
			/* Can read more than one register in sequence, and per I2C spec, the master
			 * must NACK the final byte
			*/
			if (len > 1) // Read more than one reg in sequence
			{
				i2c_master_read(handle, &buff[0], len-1, I2C_MASTER_ACK);
			}
			i2c_master_read(handle, &buff[len-1], 1, I2C_MASTER_NACK);
		}
		i2c_master_stop(handle);
		esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, handle, pdMS_TO_TICKS(timeout));
		if (ret != ESP_OK)
		{
			ESP_LOGE("IMU_TASK", "Read failed: %s.", esp_err_to_name(ret));
		}
		i2c_cmd_link_delete(handle);
	}
	
}

static void i2c_scan(void) {
    for (uint8_t addr = 1; addr < 0x7F; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI("I2C_SCAN", "Found device at 0x%02X", addr);
        }
    }
}


void imu_read_task(void * arg)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,          // SDA
        .scl_io_num = GPIO_NUM_22,          // SCL
        .sda_pullup_en = GPIO_PULLUP_ENABLE,  // enable even with 10k externals
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    ESP_ERROR_CHECK(i2c_set_timeout(I2C_NUM_0, 0xFFFFF)); // max valid HW timeout

    // Give the BNO055 firmware time to boot
    vTaskDelay(pdMS_TO_TICKS(700));
    ESP_LOGI("IMU_TASK", "Finished waiting for IMU to start");

    // Scan once to confirm the address (expect 0x28 or 0x29)
    i2c_scan();

    // Quick sanity: read CHIP_ID (0x00) should be 0xA0
    uint8_t chip_id = 0;
    i2c_read(0x00, &chip_id, 1, 200);
    ESP_LOGI("IMU_TASK", "BNO055 CHIP_ID=0x%02X (expect 0xA0)", chip_id);

    // Proceed only if ID is correct
    if (chip_id != 0xA0) {
        ESP_LOGE("IMU_TASK", "No BNO055 detected. Check address (ADR), wiring, and power.");
        vTaskDelay(portMAX_DELAY);
    }

    // Set Register page 0
    uint8_t page_zero = BNO055_PAGE_ZERO;
    i2c_write(BNO055_PAGE_ID, &page_zero, 1, 200);

    uint8_t op_mode_imu = BNO055_OPERATION_MODE_CONFIG;
    i2c_write(BNO055_OPR_MODE, &op_mode_imu, 1, 200);
    vTaskDelay(pdMS_TO_TICKS(25));

    uint8_t unit_sel = 0x10;
    i2c_write(BNO055_UNIT_SEL, &unit_sel, 1, 200);
    vTaskDelay(pdMS_TO_TICKS(25));

    // Set BNO055 to IMU mode (from CONFIGMODE it’s allowed)
    op_mode_imu = BNO055_OPERATION_MODE_IMU;
    i2c_write(BNO055_OPR_MODE, &op_mode_imu, 1, 200);
    vTaskDelay(pdMS_TO_TICKS(25));

    

    // For IMU Sensor ROS msg fields:
	//		Angular Velocity -> GYR_DATA_<axis>_LSB and GYR_DATA_<axis>_MSB
	//		Orientation Quaternion -> QUA_DATA_<dof>_LSB and QUA_DATA_<dof>_MSB
	//		Linear Acceleration -> LIA_DATA_<axis>_LSB and LIA_DATA_<axis>_MSB

    while (true)
    {
        // RAW READS
        // ========================================================
        uint8_t reg = BNO055_GYR_DATA_X_LSB;
        i2c_read(reg, raw_gyro, 6, 200);
        int16_t raw_gyro_x = (int16_t)(raw_gyro[1] << 8 | raw_gyro[0]);
        int16_t raw_gyro_y = (int16_t)(raw_gyro[3] << 8 | raw_gyro[2]);
        int16_t raw_gyro_z = (int16_t)(raw_gyro[5] << 8 | raw_gyro[4]);

        reg = BNO055_QUA_DATA_W_LSB;
        i2c_read(reg, raw_quat, 8, 200);
        int16_t raw_quat_w = (int16_t)(raw_quat[1] << 8 | raw_quat[0]);
        int16_t raw_quat_x = (int16_t)(raw_quat[3] << 8 | raw_quat[2]);
        int16_t raw_quat_y = (int16_t)(raw_quat[5] << 8 | raw_quat[4]);
        int16_t raw_quat_z = (int16_t)(raw_quat[7] << 8 | raw_quat[6]);
;
        reg = BNO055_LIA_DATA_X_LSB;
        i2c_read(reg, raw_lin, 6, 200);
        int16_t raw_lin_x = (int16_t)(raw_lin[1] << 8 | raw_lin[0]);
        int16_t raw_lin_y = (int16_t)(raw_lin[3] << 8 | raw_lin[2]);
        int16_t raw_lin_z = (int16_t)(raw_lin[5] << 8 | raw_lin[4]);
        // ========================================================
        
        // SCALED DATA
        // ========================================================
        angular_vel_data.x = ((float)raw_gyro_x) / 900.0f;
        angular_vel_data.y = ((float)raw_gyro_y) / 900.0f;
        angular_vel_data.z = ((float)raw_gyro_z) / 900.0f;

        orientation_data.w = ((float)raw_quat_w )/ 16384.0f;
        orientation_data.x = ((float)raw_quat_x )/ 16384.0f;
        orientation_data.y = ((float)raw_quat_y )/ 16384.0f;
        orientation_data.z = ((float)raw_quat_z )/ 16384.0f;

        linear_accel_data.x = ((float)raw_lin_x )/ 100.0f;
        linear_accel_data.y = ((float)raw_lin_y )/ 100.0f;
        linear_accel_data.z = ((float)raw_lin_z )/ 100.0f;
        // ========================================================

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
	
	//esp_log_level_set("*", ESP_LOG_NONE);

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

