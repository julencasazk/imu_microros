#include <string.h>
#include <stdio.h>
#include "bno055.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_system.h"


static const char *TAG = "BNO055"; // TAG for ESP32 logging
struct bno055_dev {
    i2c_port_t i2c_port;
    uint8_t addr;
    SemaphoreHandle_t lock; // Optional for thread safety 
};


bno055_dev_t* bno055_create(i2c_port_t i2c_port, uint8_t addr)
{
    bno055_dev_t *dev = calloc(1, sizeof(*dev)); // Equivalent to heap_caps_calloc(p, MALLOC_CAP_8BIT)
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate memory for BNO055 device");
        return NULL;
    }
    dev->i2c_port = i2c_port;
    dev->addr = addr & 0x7F; // 7 bit addressing mode only
    dev->lock = xSemaphoreCreateMutex(); 
    return dev;
}

void bno055_destroy(bno055_dev_t *dev){
    if (!dev){
        ESP_LOGE(TAG, "Trying to delete a NULL BNO055 device");
        return;
    }
    if (dev->lock) vSemaphoreDelete(dev->lock);
    free(dev);
}


static esp_err_t bno055_write_register(bno055_dev_t *dev, uint8_t reg, uint8_t *data, size_t len) {
    // The BNO055 expects a write secuence of DEVICE_ADDR -> REG -> DATA0 -> DATA1 -> STOP
    // so the register address to write to needs to be included in the sequence

    uint8_t complete_buff[1+len];
    complete_buff[0] = reg;
    memcpy(&complete_buff[1], data, len);
    return i2c_master_write_to_device(dev->i2c_port, dev->addr, complete_buff, len + 1, pdMS_TO_TICKS(1000));
}

static esp_err_t bno055_read_register(bno055_dev_t *dev, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(dev->i2c_port, dev->addr, &reg, 1, data, len, pdMS_TO_TICKS(1000));
}

esp_err_t bno055_set_mode(bno055_dev_t *dev, uint8_t mode)
{
    return bno055_write_register(dev, BNO055_OPR_MODE, &mode, 1); 
}

esp_err_t bno055_set_unit_selection(bno055_dev_t *dev, uint8_t unit)
{
    return bno055_write_register(dev, BNO055_UNIT_SEL, &unit, 1);
}

esp_err_t bno055_read_gyro(bno055_dev_t *dev, bno055_gyro_t* gyro_data)
{
    // TODO Read unit register and adjust division
    uint8_t raw_gyro[6] = {0};
    esp_err_t ret = bno055_read_register(dev, BNO055_GYR_DATA_X_LSB, raw_gyro, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyro data: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t raw_gyro_x = (int16_t)(raw_gyro[1] << 8 | raw_gyro[0]);
    int16_t raw_gyro_y = (int16_t)(raw_gyro[3] << 8 | raw_gyro[2]);
    int16_t raw_gyro_z = (int16_t)(raw_gyro[5] << 8 | raw_gyro[4]);

    gyro_data->x = ((float)raw_gyro_x) / 900.0f;
    gyro_data->y = ((float)raw_gyro_y) / 900.0f;
    gyro_data->z = ((float)raw_gyro_z) / 900.0f;

    return ESP_OK;
}


esp_err_t bno055_read_linear_acceleration(bno055_dev_t *dev, bno055_lin_accel_t* accel_data)
{
    // TODO Read unit register and adjust division
    uint8_t raw_lin[6] = {0};
    esp_err_t ret = bno055_read_register(dev, BNO055_LIA_DATA_X_LSB, raw_lin, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read linear acceleration data: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t raw_lin_x = (int16_t)(raw_lin[1] << 8 | raw_lin[0]);
    int16_t raw_lin_y = (int16_t)(raw_lin[3] << 8 | raw_lin[2]);
    int16_t raw_lin_z = (int16_t)(raw_lin[5] << 8 | raw_lin[4]);

    accel_data->x = ((float)raw_lin_x )/ 100.0f;
    accel_data->y = ((float)raw_lin_y )/ 100.0f;
    accel_data->z = ((float)raw_lin_z )/ 100.0f;

    return ESP_OK;
}

esp_err_t bno055_read_quaternion(bno055_dev_t *dev, bno055_quaternion_t* quat_data){
    // TODO Read unit register and adjust division
    uint8_t raw_quat[8] = {0};
    esp_err_t ret = bno055_read_register(dev, BNO055_QUA_DATA_W_LSB, raw_quat, 8);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read quaternion data: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t raw_quat_w = (int16_t)(raw_quat[1] << 8 | raw_quat[0]);
    int16_t raw_quat_x = (int16_t)(raw_quat[3] << 8 | raw_quat[2]);
    int16_t raw_quat_y = (int16_t)(raw_quat[5] << 8 | raw_quat[4]);
    int16_t raw_quat_z = (int16_t)(raw_quat[7] << 8 | raw_quat[6]);

    quat_data->w = ((float)raw_quat_w )/ 16384.0f;
    quat_data->x = ((float)raw_quat_x )/ 16384.0f;
    quat_data->y = ((float)raw_quat_y )/ 16384.0f;
    quat_data->z = ((float)raw_quat_z )/ 16384.0f;
    
    printf("\nRaw X: %f\nRaw Y: %f\nRaw Z: %f\nRaw W: %f\n", quat_data->x, quat_data->y, quat_data->z, quat_data->w);

    return ESP_OK;
}

uint8_t   bno055_get_chip_id(bno055_dev_t *dev)
{
    uint8_t chip_id = 0;
    esp_err_t err = bno055_read_register(dev, BNO055_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK || chip_id == 0) {
        ESP_LOGE(TAG, "Failed to read Chip id");
        return 0x00;
    }
    return chip_id;
}


esp_err_t bno055_init(bno055_dev_t *dev, uint8_t *opr_mode, uint8_t *unit_sel) {
    uint8_t unit_selection = (unit_sel != NULL) ? *unit_sel : 0x00; // Default to m/s^2, degrees, Celsius
    uint8_t operation_mode = (opr_mode != NULL) ? *opr_mode : BNO055_OPERATION_MODE_NDOF; // Default to NDOF mode
    uint8_t page_zero = BNO055_PAGE_ZERO;
    esp_err_t ret = bno055_write_register(dev, BNO055_PAGE_ID, &page_zero, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set page to zero: %s", esp_err_to_name(ret));
        return ret;
    }
    uint8_t config = BNO055_OPERATION_MODE_CONFIG;
    ret = bno055_write_register(dev, BNO055_OPR_MODE, &config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set operation mode to CONFIG: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(25));
    ret = bno055_write_register(dev, BNO055_UNIT_SEL, &unit_selection, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set unit selection: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(25));
    ret = bno055_write_register(dev, BNO055_OPR_MODE, &operation_mode, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set operation mode: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(25));
    ESP_LOGI(TAG, "BNO055 sucessfully configured with opr_mode: 0x%02X, and units: 0x%02X.", operation_mode, unit_selection);
    return ESP_OK;
}


