//
// Created by kok on 12.10.24.
//

#ifndef SENSOR_APP_H
#define SENSOR_APP_H

#define SENSOR_SDA_GPIO                                 33
#define SENSOR_SCL_GPIO                                 32
#define SENSOR_I2C_WRITE_ADDR                           0x29
#define SENSOR_I2C_READ_ADDR                            0x29
#define SENSOR_i2C_SCL_FREQ_HZ                          100000
#define SENSOR_OPEN_DISTANCE_MM                         250

// Sensor Registers

#define VL53L0X_REG_RESULT_RANGE_STATUS                 0x14        // Read the sensor's data

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID             0xC0        // Get model id
#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV   0x89        // Sets 2V8 mode
#define VL53L0X_REG_SYSRANGE_START                      0x00        // Set range mode
#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO        0x0A        // Configure GPIO interrupt trigger
#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH             0x84        // Configure GPIO Output on interrupt trigger
#define VL53L0X_REG_SYSTEM_THRESH_LOW                   0x0E        // Set the lower threshold

#include <esp_err.h>
#include <stdbool.h>

typedef enum {
    SENSOR_APP_POWER_1V6 = 0,                           // Standard mode (default) is used to supply from 1V6 to 1V9
    SENSOR_APP_POWER_2V8                                // 2V8 mode is used to supply from 2V6 to 3V5
} sensor_app_power_mode_e;

typedef enum {
    SENSOR_APP_SENSOR_MODE_SINGLE_SHOT = 1,             // Single-shot mode (the sensor will take one measurement and stop)
    SENSOR_APP_SENSOR_MODE_CONTIUOUS,                   // Continuous back-to-back mode (the sensor will take repeated measurements)
    SENSOR_APP_SENSOR_MODE_TIMED_CONTINUOS = 4              // Timed continuous mode
} sensor_app_range_mode_e;

typedef enum {
    SENSOR_APP_GPIO_INTR_DISABLED = 0,          // Disable GPIO interrupt
    SENSOR_APP_GPIO_INTR_LEVEL_LOW,             // Trigger interrupt below configured threshold
    SENSOR_APP_GPIO_INTR_LEVEL_HIGH,            // Trigger interrupt above configured threshold
    SENSOR_APP_GPIO_INTR_OUT_OF_WINDOW,         // Trigger interrupt when outside configured threshold range
    SENSOR_APP_GPIO_INTR_NEW_SAMPLE_READY,      // Trigger interrupt when new reading is ready
} sensor_app_gpio_intr_state_e;

typedef enum {
    SENSOR_APP_ACTIVE_STATE_LOW = 0,            // Output LOW on interrupt
    SENSOR_APP_ACTIVE_STATE_HIGH                // Output HIGH on interrupt
} sensor_app_active_state_e;

void sensor_app_init();

esp_err_t sensor_app_check_connection(bool debugLogs);

esp_err_t sensor_app_receive_data(uint8_t reg_addr, uint8_t *data, size_t len, bool debugLogs);

#endif //SENSOR_APP_H
