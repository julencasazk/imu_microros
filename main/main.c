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
#include "i2c_bus.h" 
#include "bno055.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
//#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) (void)(fn)

// PRIVATE VARS
// ========================================
const unsigned int timer_period = RCL_MS_TO_NS(10);
rcl_publisher_t imu_publisher;

static const char IMU_FRAME_ID[] = "imu_link"; 

// Remove raw byte buffers; use driver structs directly
bno055_gyro_t angular_vel_data = {0};
bno055_lin_accel_t linear_accel_data = {0};
bno055_quaternion_t orientation_data = {0};

sensor_msgs__msg__Imu imu_msg;

static uart_port_t uart_port = UART_NUM_0;

bno055_dev_t* imu_dev;
// ========================================
// END PRIVATE VARS


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

        int64_t us = esp_timer_get_time();
        imu_msg.header.stamp.sec = (int32_t)(us / 1000000LL);
        imu_msg.header.stamp.nanosec = (uint32_t)((us % 1000000LL) * 1000);

        RCCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    }
}



void micro_ros_task(void)
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



void imu_read_task(void * arg)
{

   ESP_ERROR_CHECK(i2c_bus_init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 100000));
   vTaskDelay(pdMS_TO_TICKS(1000));
   
   i2c_bus_scan();
   
   imu_dev = bno055_create(I2C_NUM_0, BNO055_I2C_ADDR_PRIMARY);


   uint8_t chip_id = bno055_get_chip_id(imu_dev);
   if (chip_id != 0xA0)
   {
       ESP_LOGE("IMU_TASK", "Chip id not correct");
   }
   

   uint8_t mode = BNO055_OPERATION_MODE_IMU;
   uint8_t units = 0x10;
   ESP_ERROR_CHECK(bno055_init(imu_dev, &mode, &units));
   vTaskDelay(pdMS_TO_TICKS(1000));
 
    while (true)
    {
        if (bno055_read_gyro(imu_dev, &angular_vel_data) != ESP_OK) {
            ESP_LOGE("IMU_TASK", "bno055_read_gyro failed");
        }

        if (bno055_read_quaternion(imu_dev, &orientation_data) != ESP_OK) {
            ESP_LOGE("IMU_TASK", "bno055_read_quaternion failed");
        }

        if (bno055_read_linear_acceleration(imu_dev, &linear_accel_data) != ESP_OK) {
            ESP_LOGE("IMU_TASK", "bno055_read_linear_acceleration failed");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    
    //esp_log_level_set("*", ESP_LOG_NONE);

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    
    xTaskCreate(micro_ros_task,
            "micro_ros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);

    xTaskCreate(imu_read_task,
            "imu_read_task",
            6144, // keep larger stack
            NULL,
            5,
            NULL);
#else
#error micro-ROS transports misconfigured
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

}

