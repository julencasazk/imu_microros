#include <string.h>
#include <i2c_bus.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

static const char *TAG = "I2C_BUS";
static i2c_port_t s_i2c_port = I2C_NUM_0;

esp_err_t i2c_bus_init(i2c_port_t i2c_num, gpio_num_t sda_io, gpio_num_t scl_io, uint32_t clk_speed)
{
    s_i2c_port = i2c_num;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = clk_speed,
    };

    esp_err_t ret = i2c_param_config(s_i2c_port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(s_i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Increase bus timeout to tolerate clock stretching
    i2c_set_timeout(s_i2c_port, 0xFFFFF);

    ESP_LOGI(TAG, "I2C init ok: port=%d SDA=%d SCL=%d Freq=%u", s_i2c_port, sda_io, scl_io, (unsigned)clk_speed);
    return ESP_OK;
}

void i2c_bus_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus on port %d...", s_i2c_port);
    for (uint8_t addr = 1; addr < 0x7F; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (!cmd) {
            ESP_LOGE(TAG, "Cmd link alloc failed");
            return;
        }
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(s_i2c_port, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI("I2C_SCAN", "Found device at 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "Scan complete");
}

