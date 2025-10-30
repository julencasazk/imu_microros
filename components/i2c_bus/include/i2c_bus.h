#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"


esp_err_t i2c_bus_init(i2c_port_t i2c_num, gpio_num_t sda_io, gpio_num_t scl_io, uint32_t clk_speed);
void i2c_bus_scan(void);

#endif
