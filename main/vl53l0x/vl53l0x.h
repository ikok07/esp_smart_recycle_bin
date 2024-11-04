//
// Created by Kok on 11/4/24.
//

#ifndef VL53L0X_H
#define VL53L0X_H

#include "esp_err.h"

// Sensor known values
#define VL53L0X_MODEL_ID                                                 0xEE

// Sensor Registers

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID                             0xC0        // Get model id
#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV                   0x89        // Sets 2V8 mode
#define VL53L0X_REG_MSRC_CONFIG_CONTROL                                 0x60        // Sets a threshold for the signal rate (intensity of reflected signal) received by the sensor
#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT         0x44        // Defines the minimum return signal rate limit for valid final range measurements.
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG                              0x01        // Controls the sequence of steps the VL53L0X sensor follows during initialization and ranging
#define VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET                    0x4F        // Sets the start offset for enabling dynamic SPADs (Single Photon Avalanche Diodes) that the sensor uses for measurement.
#define VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD                 0x4E        // Specifies the number of reference SPADs to be enabled for dynamic SPAD allocation.
#define VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT                   0xB6        // Selects the start position of the reference SPADs used in the global configuration.
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR                              0x0B        // Clears the system interrupt flag
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS                             0x13        // Holds the interrupt status after a measurement
#define VL53L0X_REG_RESULT_RANGE_STATUS                                 0x14        // Read the sensor's data
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0                    0xB0        // Controls the SPAD enable configuration for the reference array
#define VL53L0X_REG_SYSRANGE_START                                      0x00        // Set range mode
#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO                        0x0A        // Configure GPIO interrupt trigger
#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH                             0x84        // Configure GPIO Output on interrupt trigger
#define VL53L0X_REG_SYSTEM_THRESH_LOW                                   0x0E        // Set the lower threshold

// Sequence Steps

#define RANGE_SEQUENCE_STEP_TCC                                         0x10        // Target CentreCheck (Check to see if the target is centered)
#define RANGE_SEQUENCE_STEP_MSRC                                        0x04        // Minimum Signal Rate Check (Check if the signal rate is below a certain threshold, return error value if it is)
#define RANGE_SEQUENCE_STEP_DSS                                         0x28        // Dynamic SPAD selection (Dynamically select the active SPADs to avoid saturating from too much light)
#define RANGE_SEQUENCE_STEP_PRE_RANGE                                   0x40
#define RANGE_SEQUENCE_STEP_FINAL_RANGE                                 0x80

typedef enum {
    VL53L0X_POWER_1V6 = 0,                           // Standard mode (default) is used to supply from 1V6 to 1V9
    VL53L0X_POWER_2V8                                // 2V8 mode is used to supply from 2V6 to 3V5
} vl53l0x_power_mode_e;

typedef enum {
    VL53L0X_SENSOR_MODE_SINGLE_SHOT = 1,             // Single-shot mode (the sensor will take one measurement and stop)
    VL53L0X_SENSOR_MODE_CONTIUOUS,                   // Continuous back-to-back mode (the sensor will take repeated measurements)
    VL53L0X_SENSOR_MODE_TIMED_CONTINUOS = 4              // Timed continuous mode
} vl53l0x_range_mode_e;

typedef enum {
    VL53L0X_GPIO_INTR_DISABLED = 0,          // Disable GPIO interrupt
    VL53L0X_GPIO_INTR_LEVEL_LOW,             // Trigger interrupt below configured threshold
    VL53L0X_GPIO_INTR_LEVEL_HIGH,            // Trigger interrupt above configured threshold
    VL53L0X_GPIO_INTR_OUT_OF_WINDOW,         // Trigger interrupt when outside configured threshold range
    VL53L0X_GPIO_INTR_NEW_SAMPLE_READY,      // Trigger interrupt when new reading is ready
} vl53l0x_gpio_intr_state_e;

typedef enum {
    VL53L0X_ACTIVE_STATE_LOW = 0,            // Output LOW on interrupt
    VL53L0X_ACTIVE_STATE_HIGH                // Output HIGH on interrupt
} vl53l0x_active_state_e;

typedef enum
{
    VL53L0X_CALIBRATION_TYPE_VHV,
    VL53L0X_CALIBRATION_TYPE_PHASE
} vl53l0x_calibration_type_t;

typedef struct {
    uint8_t sda_gpio;
    uint8_t scl_gpio;
    uint8_t i2c_addr;
    uint32_t i2c_freq_hz;
} vl53l0x_init_config_t;

esp_err_t vl53l0x_init(vl53l0x_init_config_t config);
esp_err_t vl53l0x_check_connection();


#endif //VL53L0X_H
