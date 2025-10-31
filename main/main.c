#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/bool.h>
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


extern s8 bno055_esp32_bind(i2c_port_t port, u8 dev_addr_7bit);
extern struct bno055_t bno055;

// PRIVATE VARS
// ========================================
const unsigned int timer_period = RCL_MS_TO_NS(5);
rcl_publisher_t imu_publisher;
rcl_publisher_t crash_happened_publisher;

static const char IMU_FRAME_ID[] = "imu_link"; 

static float ang_vel_x = 0, ang_vel_y = 0, ang_vel_z = 0;        // rad/s
static float lin_acc_x = 0, lin_acc_y = 0, lin_acc_z = 0;        // m/s^2
static float quat_w = 1, quat_x = 0, quat_y = 0, quat_z = 0;     // unit quaternion

sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Bool crash_happened_msg;


static TaskHandle_t crash_task_handle = NULL;
static SemaphoreHandle_t rcl_mutex = NULL;
bool interrupt_happened = false;
static volatile bool bno_ready = false;  // set true after BNO055 init is done
// Datasheet: SYS_TRIGGER.RST_INT is bit 6
#ifndef BNO055_SYS_TRIGGER_RST_INT_BIT
#define BNO055_SYS_TRIGGER_RST_INT_BIT (1u << 6)
#endif
// ========================================
// END PRIVATE VARS

static void reset_int(void)
{
    if (!bno_ready || !bno055.bus_read || !bno055.bus_write) {
        return;
    }

    uint8_t intr_status = 0x00;
    (void)bno055.bus_read(bno055.dev_addr, BNO055_INTR_STAT_ADDR, &intr_status, 1);

    uint8_t sys_trig = 0x00;
    if (bno055.bus_read(bno055.dev_addr, BNO055_SYS_TRIGGER_ADDR, &sys_trig, 1) == BNO055_SUCCESS) {
        sys_trig |= BNO055_SYS_TRIGGER_RST_INT_BIT;
        (void)bno055.bus_write(bno055.dev_addr, BNO055_SYS_TRIGGER_ADDR, &sys_trig, 1);
    }
    ESP_LOGI("GPIO_EVT", "INT_STAT=0x%02X (RST_INT issued)", intr_status);
}


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (crash_task_handle)
    {
        vTaskNotifyGiveFromISR(crash_task_handle, &xHigherPriorityTaskWoken);
    }
    
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
    
}

static void gpio_ext_interrupt_init(void)
{
    ESP_LOGI("GPIO_EXTI_INIT", "Initializing GPIO Pin 19 in exti edge mode...");
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_19),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,      
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE         
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    int lvl = gpio_get_level(GPIO_NUM_19);
    ESP_LOGI("GPIO_EXTI_INIT", "GPIO19 initial level=%d", lvl);

    static bool isr_service_installed = false;
    if (!isr_service_installed) {
        ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
        ESP_LOGI("GPIO_EXTI_INIT", "GPIO ISR service installed.");
        isr_service_installed = true;
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_NUM_19, gpio_isr_handler, (void*) GPIO_NUM_19));
}


void gpio_exti_task(void* arg)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int level = gpio_get_level(GPIO_NUM_19);
        ESP_LOGI("GPIO_EVT", "INT edge detected, GPIO19 level=%d", level);

        if (!crash_happened_publisher.impl)
        {
            ESP_LOGI("GPIO_EVT", "Ignoring interrupt, publisher not ready yet.");
            reset_int(); // still clear so we can detect the next rising edge
            continue;
        }
        
        ESP_LOGI("GPIO_EVT", "Crash interrupt detected!");

        std_msgs__msg__Bool msg = {
            .data = true
        };

        xSemaphoreTake(rcl_mutex, portMAX_DELAY);
        rcl_ret_t rc = rcl_publish(&crash_happened_publisher, &msg, NULL);
        xSemaphoreGive(rcl_mutex);

        reset_int();
        ESP_LOGI("GPIO_EVT", "INT pin cleared, GPIO19 level=%d", gpio_get_level(GPIO_NUM_19));
        
        if(rc != RCL_RET_OK){
            ESP_LOGE("GPIO_EVT", "Publish failed: %d", (int)rc);
        }
    }
    
}

// CALLBACK FUNCTIONS
// -------------------------------
static void timer_cb(rcl_timer_t* timer, int64_t last_call_time)
{
    if (timer != NULL)
    {
        imu_msg.angular_velocity = (geometry_msgs__msg__Vector3){
            .x = ang_vel_x,
            .y = ang_vel_y,
            .z = ang_vel_z
        };

        imu_msg.linear_acceleration = (geometry_msgs__msg__Vector3){
            .x = lin_acc_x,
            .y = lin_acc_y,
            .z = lin_acc_z
        };
        
        imu_msg.orientation = (geometry_msgs__msg__Quaternion){
            .w = quat_w,
            .x = quat_x,
            .y = quat_y,
            .z = quat_z
        };

        int64_t us = esp_timer_get_time();
        imu_msg.header.stamp.sec = (int32_t)(us / 1000000LL);
        imu_msg.header.stamp.nanosec = (uint32_t)((us % 1000000LL) * 1000);

        RCCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    }
}


void micro_ros_task(void* arg)
{
    
    uart_port_t uart_port = UART_NUM_0;
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

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // TIMERS
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, timer_period, timer_cb));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // PUBLISHERS
    RCCHECK(rclc_publisher_init_best_effort(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_data"
    ));
   
    RCCHECK(rclc_publisher_init_best_effort(
        &crash_happened_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "crash_happened"
    ));


    // Initialize message and set fixed frame_id
    sensor_msgs__msg__Imu__init(&imu_msg);
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, IMU_FRAME_ID);

    // Unknown covariances: follow ROS convention ([0] = -1, rest 0)
    imu_msg.orientation_covariance[0] = -1.0;
    imu_msg.angular_velocity_covariance[0] = -1.0;
    imu_msg.linear_acceleration_covariance[0] = -1.0;

    while(1){
        xSemaphoreTake(rcl_mutex, portMAX_DELAY);
        RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
        xSemaphoreGive(rcl_mutex);

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
    RCCHECK(rcl_publisher_fini(&crash_happened_publisher, &node));
    sensor_msgs__msg__Imu__fini(&imu_msg);

    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rcl_node_fini(&node));

      vTaskDelete(NULL);
}



void imu_read_task(void * arg)
{
   ESP_ERROR_CHECK(i2c_bus_init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 400000));
   vTaskDelay(pdMS_TO_TICKS(1000));
   
   i2c_bus_scan();

   // Bind Bosch API to ESP32 I2C and device address (0x28 default, 0x29 if ADR high)
   (void)bno055_esp32_bind(I2C_NUM_0, BNO055_I2C_ADDR1);

   if (bno055_init(&bno055) != BNO055_SUCCESS) {
       ESP_LOGE("IMU_TASK", "bno055_init failed");
       vTaskDelete(NULL);
       return;
   }

   // Verify chip ID
   uint8_t chip_id = 0x00;
   if (bno055.bus_read) {
       bno055.bus_read(bno055.dev_addr, BNO055_CHIP_ID_ADDR, &chip_id, 1);
   }
   if (chip_id != 0xA0) {
       ESP_LOGE("IMU_TASK", "Chip id not correct: 0x%02X", chip_id);
   }

   (void)bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
   vTaskDelay(pdMS_TO_TICKS(10));
   (void)bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
   vTaskDelay(pdMS_TO_TICKS(25));

   // Units: SI (m/s^2, rad/s)
   (void)bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
   (void)bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);


   // Clear any stale status
   reset_int();

   // High-G config 
   s8 rc;
   rc = bno055_set_intr_accel_high_g(BNO055_BIT_ENABLE); ESP_LOGI("IMU_TASK", "set_intr_accel_high_g rc=%d", rc);
   rc = bno055_set_intr_mask_accel_high_g(BNO055_BIT_ENABLE); ESP_LOGI("IMU_TASK", "set_intr_mask_accel_high_g rc=%d", rc);
   rc = bno055_set_accel_high_g_thres(0xC0); ESP_LOGI("IMU_TASK", "set_accel_high_g_thres rc=%d", rc);
   rc = bno055_set_accel_high_g_durn(0x03);  ESP_LOGI("IMU_TASK", "set_accel_high_g_durn rc=%d", rc);
   rc = bno055_set_accel_high_g_axis_enable(BNO055_ACCEL_HIGH_G_X_AXIS, BNO055_BIT_ENABLE); ESP_LOGI("IMU_TASK", "axis X rc=%d", rc);
   rc = bno055_set_accel_high_g_axis_enable(BNO055_ACCEL_HIGH_G_Y_AXIS, BNO055_BIT_ENABLE); ESP_LOGI("IMU_TASK", "axis Y rc=%d", rc);
   rc = bno055_set_accel_high_g_axis_enable(BNO055_ACCEL_HIGH_G_Z_AXIS, BNO055_BIT_DISABLE); ESP_LOGI("IMU_TASK", "axis Z rc=%d (disabled)", rc);

    (void)bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI("IMU_TASK", "Finished configuring interrupt on BNO055!");
  bno_ready = true;

   while (true)
   {
        // Gyro in rad/s
        struct bno055_gyro_double_t d_gyro = {0};
        if (bno055_convert_double_gyro_xyz_rps(&d_gyro) == BNO055_SUCCESS) {
            ang_vel_x = (float)d_gyro.x;
            ang_vel_y = (float)d_gyro.y;
            ang_vel_z = (float)d_gyro.z;
        } else {
            ESP_LOGE("IMU_TASK", "bno055_convert_double_gyro_xyz_rps failed");
        }

        // Linear acceleration in m/s^2
        struct bno055_linear_accel_double_t d_lin = {0};
        if (bno055_convert_double_linear_accel_xyz_msq(&d_lin) == BNO055_SUCCESS) {
            lin_acc_x = (float)d_lin.x;
            lin_acc_y = (float)d_lin.y;
            lin_acc_z = (float)d_lin.z;
        } else {
            ESP_LOGE("IMU_TASK", "bno055_convert_double_linear_accel_xyz_msq failed");
        }

        // Quaternion raw -> normalized (scale 1/16384 per datasheet)
        struct bno055_quaternion_t q = {0};
        if (bno055_read_quaternion_wxyz(&q) == BNO055_SUCCESS) {
            const float scale = 1.0f / 16384.0f;
            quat_w = (float)q.w * scale;
            quat_x = (float)q.x * scale;
            quat_y = (float)q.y * scale;
            quat_z = (float)q.z * scale;
            
        } else {
            ESP_LOGE("IMU_TASK", "bno055_read_quaternion_wxyz failed");
        }

        vTaskDelay(pdMS_TO_TICKS(4));
   }
}

static void bno055_int_debug_task(void *arg)
{
    for (;;)
    {
        if (!bno_ready) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        int lvl = gpio_get_level(GPIO_NUM_19);
        uint8_t istat = 0;
        if (bno055.bus_read) {
            bno055.bus_read(bno055.dev_addr, BNO055_INTR_STAT_ADDR, &istat, 1);
        }
        ESP_LOGI("INT_DBG", "GPIO19=%d, INT_STAT=0x%02X", lvl, istat);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    rcl_mutex = xSemaphoreCreateMutex();
    configASSERT(rcl_mutex);

    // Disabling UART logging avoids stealing bandwidth
    // esp_log_level_set("*", ESP_LOG_NONE);
    // From my testing, it makes no difference
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)

    xTaskCreate(micro_ros_task,
            "micro_ros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
        

    xTaskCreate(imu_read_task,
            "imu_read_task",
            6144, 
            NULL,
            5,
            NULL);

    xTaskCreate(gpio_exti_task,
            "gpio_exti_task",
            2048,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO + 2,
            &crash_task_handle);
    
    gpio_ext_interrupt_init();
    xTaskCreate(bno055_int_debug_task, "bno055_int_debug_task", 2048, NULL, 4, NULL);
#else
#error micro-ROS transports misconfigured
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

}

