//
// Created by kok on 12.10.24.
//

#include "sensor_app.h"

#include <esp_log.h>
#include <tasks.h>
#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <pwm_app/pwm_app.h>

static const char TAG[] = "sensor_app";

i2c_master_bus_handle_t sensor_app_i2c_bus_handle;
i2c_master_dev_handle_t sensor_app_i2c_device_handle;

static void sensor_app_task() {
    esp_err_t err;
    while (1) {
        if ((err = sensor_app_check_connection()) != ESP_OK) {
            ESP_LOGE(TAG, "Sensor is not connected! Setting servo to closed position!");
            pwm_app_send_message(PWM_APP_MSG_CLOSE_SERVO);
            continue;
        }
        uint8_t distance;
        if ((err = sensor_app_receive_data(&distance)) != ESP_OK) {
            ESP_LOGE(TAG, "Could not fetch data from sensor! Setting servo to closed position!");
            pwm_app_send_message(PWM_APP_MSG_CLOSE_SERVO);
            continue;
        }
        if (distance < SENSOR_OPEN_DISTANCE) pwm_app_send_message(PWM_APP_MSG_OPEN_SERVO);
        else pwm_app_send_message(PWM_APP_MSG_CLOSE_SERVO);
    }
}

static void sensor_app_i2c_configure_bus() {
    ESP_LOGI(TAG, "Configuring I2C Master Bus");
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SENSOR_SDA_GPIO,
        .scl_io_num = SENSOR_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_APB, // prevent system from changing APB freq before going into light sleed mode,
        .glitch_ignore_cnt = 7, // increase if data is unstable
        .flags = {.enable_internal_pullup = true} // Pull up should be from external resistor
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &sensor_app_i2c_bus_handle));
}

static void sensor_app_i2c_configure_device() {
    ESP_LOGI(TAG, "Configuring I2C Device");
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SENSOR_I2C_ADDR,
        .scl_speed_hz = SENSOR_i2C_SCL_FREQ_HZ,
        .scl_wait_us = 0
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(sensor_app_i2c_bus_handle, &device_config, &sensor_app_i2c_device_handle));
}

void sensor_app_init() {
    ESP_LOGI(TAG, "Initializing sensor application");

    // Configure master bus
    sensor_app_i2c_configure_bus();

    // Configure device
    sensor_app_i2c_configure_device();

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

esp_err_t sensor_app_check_connection() {
    return i2c_master_probe(sensor_app_i2c_bus_handle, SENSOR_I2C_ADDR, portMAX_DELAY);
}

esp_err_t sensor_app_receive_data(uint8_t *data) {
    return i2c_master_receive(sensor_app_i2c_device_handle, data, sizeof(uint8_t), portMAX_DELAY);
}
