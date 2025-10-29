#ifndef BNO055_H
#define BNO055_H


#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

// ============================================================================
// Bosch BNO055 9-DOF IMU Register Address Map
// Source: BOSCH BNO055 Datasheet rev. 1.8 October 2021, Doc.Num: BST-BNO055-DS000-18 
// ============================================================================

typedef enum
{
    // PAGE 0
    BNO055_CHIP_ID              = 0x00,
    BNO055_ACC_ID               = 0x01,
    BNO055_MAG_ID               = 0x02,
    BNO055_GYR_ID               = 0x03,
    BNO055_SW_REV_ID_LSB        = 0x04,
    BNO055_SW_REV_ID_MSB        = 0x05,
    BNO055_BL_REV_ID            = 0x06,
    BNO055_PAGE_ID              = 0x07,

    BNO055_ACC_DATA_X_LSB       = 0x08,
    BNO055_ACC_DATA_X_MSB       = 0x09,
    BNO055_ACC_DATA_Y_LSB       = 0x0A,
    BNO055_ACC_DATA_Y_MSB       = 0x0B,
    BNO055_ACC_DATA_Z_LSB       = 0x0C,
    BNO055_ACC_DATA_Z_MSB       = 0x0D,

    BNO055_MAG_DATA_X_LSB       = 0x0E,
    BNO055_MAG_DATA_X_MSB       = 0x0F,
    BNO055_MAG_DATA_Y_LSB       = 0x10,
    BNO055_MAG_DATA_Y_MSB       = 0x11,
    BNO055_MAG_DATA_Z_LSB       = 0x12,
    BNO055_MAG_DATA_Z_MSB       = 0x13,

    BNO055_GYR_DATA_X_LSB       = 0x14,
    BNO055_GYR_DATA_X_MSB       = 0x15,
    BNO055_GYR_DATA_Y_LSB       = 0x16,
    BNO055_GYR_DATA_Y_MSB       = 0x17,
    BNO055_GYR_DATA_Z_LSB       = 0x18,
    BNO055_GYR_DATA_Z_MSB       = 0x19,

    BNO055_EUL_HEADING_LSB      = 0x1A,
    BNO055_EUL_HEADING_MSB      = 0x1B,
    BNO055_EUL_ROLL_LSB         = 0x1C,
    BNO055_EUL_ROLL_MSB         = 0x1D,
    BNO055_EUL_PITCH_LSB        = 0x1E,
    BNO055_EUL_PITCH_MSB        = 0x1F,

    BNO055_QUA_DATA_W_LSB       = 0x20,
    BNO055_QUA_DATA_W_MSB       = 0x21,
    BNO055_QUA_DATA_X_LSB       = 0x22,
    BNO055_QUA_DATA_X_MSB       = 0x23,
    BNO055_QUA_DATA_Y_LSB       = 0x24,
    BNO055_QUA_DATA_Y_MSB       = 0x25,
    BNO055_QUA_DATA_Z_LSB       = 0x26,
    BNO055_QUA_DATA_Z_MSB       = 0x27,

    BNO055_LIA_DATA_X_LSB       = 0x28,
    BNO055_LIA_DATA_X_MSB       = 0x29,
    BNO055_LIA_DATA_Y_LSB       = 0x2A,
    BNO055_LIA_DATA_Y_MSB       = 0x2B,
    BNO055_LIA_DATA_Z_LSB       = 0x2C,
    BNO055_LIA_DATA_Z_MSB       = 0x2D,

    BNO055_GRV_DATA_X_LSB       = 0x2E,
    BNO055_GRV_DATA_X_MSB       = 0x2F,
    BNO055_GRV_DATA_Y_LSB       = 0x30,
    BNO055_GRV_DATA_Y_MSB       = 0x31,
    BNO055_GRV_DATA_Z_LSB       = 0x32,
    BNO055_GRV_DATA_Z_MSB       = 0x33,

    BNO055_TEMP                 = 0x34,
    BNO055_CALIB_STAT           = 0x35,
    BNO055_ST_RESULT            = 0x36,
    BNO055_INT_STA              = 0x37,
    BNO055_SYS_CLK_STATUS       = 0x38,
    BNO055_SYS_STATUS           = 0x39,
    BNO055_SYS_ERR              = 0x3A,
    BNO055_UNIT_SEL             = 0x3B,
    BNO055_OPR_MODE             = 0x3D,
    BNO055_PWR_MODE             = 0x3E,
    BNO055_SYS_TRIGGER          = 0x3F,
    BNO055_TEMP_SOURCE          = 0x40,
    BNO055_AXIS_MAP_CONFIG      = 0x41,
    BNO055_AXIS_MAP_SIGN        = 0x42,

    BNO055_ACC_OFFSET_X_LSB     = 0x55,
    BNO055_ACC_OFFSET_X_MSB     = 0x56,
    BNO055_ACC_OFFSET_Y_LSB     = 0x57,
    BNO055_ACC_OFFSET_Y_MSB     = 0x58,
    BNO055_ACC_OFFSET_Z_LSB     = 0x59,
    BNO055_ACC_OFFSET_Z_MSB     = 0x5A,

    BNO055_MAG_OFFSET_X_LSB     = 0x5B,
    BNO055_MAG_OFFSET_X_MSB     = 0x5C,
    BNO055_MAG_OFFSET_Y_LSB     = 0x5D,
    BNO055_MAG_OFFSET_Y_MSB     = 0x5E,
    BNO055_MAG_OFFSET_Z_LSB     = 0x5F,
    BNO055_MAG_OFFSET_Z_MSB     = 0x60,

    BNO055_GYR_OFFSET_X_LSB     = 0x61,
    BNO055_GYR_OFFSET_X_MSB     = 0x62,
    BNO055_GYR_OFFSET_Y_LSB     = 0x63,
    BNO055_GYR_OFFSET_Y_MSB     = 0x64,
    BNO055_GYR_OFFSET_Z_LSB     = 0x65,
    BNO055_GYR_OFFSET_Z_MSB     = 0x66,

    BNO055_ACC_RADIUS_LSB       = 0x67,
    BNO055_ACC_RADIUS_MSB       = 0x68,
    BNO055_MAG_RADIUS_LSB       = 0x69,
    BNO055_MAG_RADIUS_MSB       = 0x6A,
    
    // PAGE 1


    BNO055_ACC_CONFIG           = 0x08,
    BNO055_MAG_CONFIG           = 0x09,
    BNO055_GYR_CONFIG_0         = 0x0A,
    BNO055_GYR_CONFIG_1         = 0x0B,
    BNO055_ACC_SLEEP_CONFIG     = 0x0C,
    BNO055_GYR_SLEEP_CONFIG     = 0x0D,
    BNO055_INT_MSK              = 0x0F,
    BNO055_INT_EN               = 0x10,
    BNO055_ACC_AM_THRES         = 0x11,
    BNO055_ACC_INT_SETTINGS     = 0x12,
    BNO055_ACC_HG_DURATION      = 0x13,
    BNO055_ACC_HG_THRES         = 0x14,
    BNO055_ACC_NM_THRES         = 0x15,
    BNO055_ACC_NM_SET           = 0x16,
    BNO055_GYR_INT_SETTING      = 0x17,
    BNO055_GYR_HR_X_SET         = 0x18,
    BNO055_GYR_DUR_X            = 0x19,
    BNO055_GYR_HR_Y_SET         = 0x1A,
    BNO055_GYR_DUR_Y            = 0x1B,
    BNO055_GYR_HR_Z_SET         = 0x1C,
    BNO055_GYR_DUR_Z            = 0x1D,
    BNO055_GYR_AM_THRES         = 0x1E,
    BNO055_GYR_AM_SET           = 0x1F,
    BNO055_UNIQUE_ID_START      = 0x50,   // through 0x5F
} bno055_reg_t;
// ===========================================
// END REGISTER MAP

// BNO055 OPERATION MODES
typedef enum 
{
    BNO055_OPERATION_MODE_CONFIG = 0x00,
    BNO055_OPERATION_MODE_ACCONLY = 0x01,
    BNO055_OPERATION_MODE_MAGONLY = 0x02,
    BNO055_OPERATION_MODE_GYRONLY = 0x03,
    BNO055_OPERATION_MODE_ACCMAG = 0x04,
    BNO055_OPERATION_MODE_ACCGYRO = 0x05,
    BNO055_OPERATION_MODE_MAGGYRO = 0x06,
    BNO055_OPERATION_MODE_AMG = 0x07,
    BNO055_OPERATION_MODE_IMU = 0x08,
    BNO055_OPERATION_MODE_COMPASS = 0x09,
    BNO055_OPERATION_MODE_M4G = 0x0A,
    BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
    BNO055_OPERATION_MODE_NDOF = 0x0C
} bno055_opmode_t;

// Page Number, multiplexes register addresses
typedef enum {
    BNO055_PAGE_ZERO = 0x00,
    BNO055_PAGE_ONE = 0x01
} bno055_page_num;

// READING DATA TYPES
// ===========================================
typedef struct
{
    float x;
    float y;
    float z;
} bno055_gyro_t;

typedef struct
{
    float x;
    float y;
    float z;
    float w;
} bno055_quaternion_t;

typedef struct
{
    float x;
    float y;
    float z;
} bno055_lin_accel_t;

typedef struct bno055_dev bno055_dev_t; // Defined in source file
// ===============================================
// DATA TYPES END

// Both posible addresses, depends on COM3/I2C_ADDR pin
// or ADR pin on Adafruit breakout board
#define BNO055_I2C_ADDR_PRIMARY   0x28
#define BNO055_I2C_ADDR_ALTERNATE 0x29

bno055_dev_t *bno055_create(i2c_port_t i2c_port, uint8_t addr);
void bno055_destroy(bno055_dev_t *dev);

/**
 * @brief Initializes a BNO055 device to default values
 * 
 * This function initializes the BNO055 device by setting it to page 0,
 * configuring the operation mode to CONFIG, setting the unit selection,
 * and finally switching to the desired operation mode.
 * 
 * @param dev Pointer to the BNO055 device handle
 * @param opr_mode Pointer to the desired operation mode. If NULL, defaults to NDOF mode.
 * @param unit_sel Pointer to the desired unit selection. If NULL, defaults to m/s^2, degrees, Celsius.
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t bno055_init(bno055_dev_t *dev, uint8_t *opr_mode, uint8_t *unit_sel);

/**
 * @brief Sets the operation mode of the BNO055 device
 * 
 * @param dev Pointer to the BNO055 device handle
 * @param mode The desired operation mode to set
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t bno055_set_mode(bno055_dev_t *dev, uint8_t mode);

/**
 * @brief Reads gyroscope data from the BNO055 device
 *
 * @param dev Pointer to the BNO055 device handle
 * @param gyro_data Pointer to a bno055_gyro_t structure to store the gyroscope data
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t bno055_read_gyro(bno055_dev_t *dev, bno055_gyro_t* gyro_data);
/**
 * @brief Reads linear acceleration data from the BNO055 device
 *
 * @param dev Pointer to the BNO055 device handle
 * @param accel_data Pointer to a bno055_lin_accel_t structure to store the linear acceleration data
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t bno055_read_linear_acceleration(bno055_dev_t *dev, bno055_lin_accel_t* accel_data);

/**
 * @brief Reads quaternion data from the BNO055 device
 *
 * @param dev Pointer to the BNO055 device handle
 * @param quat_data Pointer to a bno055_quaternion_t structure to store the quaternion
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t bno055_read_quaternion(bno055_dev_t *dev, bno055_quaternion_t* quat_data);

/**
 * @brief Retrieves the chip ID of the BNO055 device
 *
 * @param dev Pointer to the BNO055 device handle
 *
 * @return uint8_t The chip ID value, or 0x00 if reading fails
 * 
 */
uint8_t bno055_get_chip_id(bno055_dev_t *dev);

/**
 * @brief Sets the unit selection for the BNO055 device
 *
 * @param dev Pointer to the BNO055 device handle
 * @param unit The unit selection value to set
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t bno055_set_unit_selection(bno055_dev_t *dev, uint8_t unit);


#endif // BNO055_H
