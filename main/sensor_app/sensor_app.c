//
// Created by kok on 12.10.24.
//

#include "sensor_app.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <pwm_app/pwm_app.h>

#include "tasks.h"
#include "vl53l0x/vl53l0x.h"

// TODO: Develop custom driver (ref: https://www.artfulbytes.com/vl53l0x-post)

static const char TAG[] = "sensor_app";

static void sensor_app_task() {
    esp_err_t err;
    while (1) {
        if ((err = vl53l0x_check_connection(false)) != ESP_OK) {
            ESP_LOGE(TAG, "Sensor is not connected! Setting servo to closed position!");
            pwm_app_send_message(PWM_APP_MSG_CLOSE_SERVO);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        // uint8_t distance_data[2];
        // if ((err = vl53l0x_i2c_read(VL53L0X_REG_RESULT_RANGE_STATUS, distance_data, sizeof(distance_data), false)) != ESP_OK) {
        //     ESP_LOGE(TAG, "Could not fetch data from sensor! Setting servo to closed position!");
        //     pwm_app_send_message(PWM_APP_MSG_CLOSE_SERVO);
        //     continue;
        // }
        // uint16_t distance = (distance_data[0] << 8) | distance_data[1];
        // ESP_LOGI(TAG, "Distance: %d", distance);
        // if (distance < SENSOR_OPEN_DISTANCE_MM) pwm_app_send_message(PWM_APP_MSG_OPEN_SERVO);
        // else pwm_app_send_message(PWM_APP_MSG_CLOSE_SERVO);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void sensor_app_init() {
    ESP_LOGI(TAG, "Initializing sensor application");

    const vl53l0x_init_config_t init_config = {
        .scl_gpio = SENSOR_SCL_GPIO,
        .sda_gpio = SENSOR_SDA_GPIO,
        .i2c_addr = SENSOR_I2C_WRITE_ADDR,
        .i2c_freq_hz = SENSOR_i2C_SCL_FREQ_HZ
    };
    ESP_ERROR_CHECK(vl53l0x_init(init_config));

    xTaskCreatePinnedToCore(
        sensor_app_task,
        "sensor_app_task",
        SENSOR_APP_TASK_STACK_SIZE,
        NULL,
        SENSOR_APP_TASK_PRIORITY,
        NULL,
        SENSOR_APP_TASK_CORE_ID
    );

    ESP_LOGI(TAG, "Sensor application initialized");
}

// const VL53L0X_DEV dev = malloc(sizeof(VL53L0X_DEV));
// dev->i2c_address = SENSOR_I2C_ADDR;
// dev->i2c_port_num = I2C_NUM_0;
// dev->scl_speed_hz = SENSOR_i2C_SCL_FREQ_HZ;
// dev->scl_wait_us = 0;
// dev->scl_gpio = SENSOR_SCL_GPIO;
// dev->sda_gpio = SENSOR_SDA_GPIO;
// dev->clk_src = I2C_CLK_SRC_APB;
// dev->glitch_ignore_cnt = 7;
// dev->enable_internal_pullup = true; // TODO: set to false for production
//
// VL53L0X_Error sensor_err = VL53L0X_DataInit(dev);
// if (sensor_err != VL53L0X_ERROR_NONE) {
//     ESP_LOGE(TAG, "Failed to initialize sensor! VL53L0X_DataInit");
//     return;
// }
// sensor_err = VL53L0X_StaticInit(dev);
// if (sensor_err != VL53L0X_ERROR_NONE) {
//     ESP_LOGE(TAG, "Failed to initialize sensor! VL53L0X_StaticInit");
//     return;
// }
// sensor_err = VL53L0X_SetOffsetCalibrationDataMicroMeter(dev, 0);
// if (sensor_err != VL53L0X_ERROR_NONE) {
//     ESP_LOGE(TAG, "Failed to initialize sensor! VL53L0X_SetOffsetCalibrationDataMicroMeter");
//     return;
// }
// sensor_err = VL53L0X_StartMeasurement(dev);
// if (sensor_err != VL53L0X_ERROR_NONE) {
//     ESP_LOGE(TAG, "Failed to start sensor! VL53L0X_StartMeasurement");
//     return;
// }
