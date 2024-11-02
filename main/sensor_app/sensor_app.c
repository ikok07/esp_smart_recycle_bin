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
// #include "vl53l0x_api.h"

static const char TAG[] = "sensor_app";

static esp_err_t sensor_app_i2c_write(const uint8_t reg_addr, const uint8_t data);

void i2c_scanner() {
    i2c_cmd_handle_t cmd;
    esp_err_t err;
    printf("Scanning...\n");
    for (int i = 1; i < 127; i++) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            printf("Found device at 0x%02x\n", i);
        }
    }
    printf("Scan complete.\n");
}

static void sensor_app_task() {
    esp_err_t err;
    while (1) {
        if ((err = sensor_app_check_connection(false)) != ESP_OK) {
            ESP_LOGE(TAG, "Sensor is not connected! Setting servo to closed position!");
            pwm_app_send_message(PWM_APP_MSG_CLOSE_SERVO);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        uint8_t distance_data;
        if ((err = sensor_app_receive_data(VL53L0X_REG_RESULT_RANGE_STATUS, &distance_data, sizeof(distance_data), false)) != ESP_OK) {
            ESP_LOGE(TAG, "Could not fetch data from sensor! Setting servo to closed position!");
            pwm_app_send_message(PWM_APP_MSG_CLOSE_SERVO);
            continue;
        }
        // uint16_t distance = (distance_data[1] << 8) | distance_data[0];
        ESP_LOGI(TAG, "Distance: %d", distance_data);
        if (distance_data < SENSOR_OPEN_DISTANCE_MM) pwm_app_send_message(PWM_APP_MSG_OPEN_SERVO);
        else pwm_app_send_message(PWM_APP_MSG_CLOSE_SERVO);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void sensor_app_i2c_configure_driver() {
    ESP_LOGI(TAG, "Configuring I2C Driver");
    const i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SENSOR_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = SENSOR_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = SENSOR_i2C_SCL_FREQ_HZ,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));

    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C driver successfully configured");
}

static void sensor_app_i2c_configure_device() {
    esp_err_t check_err;
    while ((check_err = sensor_app_check_connection(true)) != ESP_OK) {
        ESP_LOGE(TAG, "Sensor is not connected! Waiting for connection... (Error: %s)", esp_err_to_name(check_err));
        vTaskDelay(10);
    }

    // Enable 2V8 mode
    ESP_LOGI(TAG, "Setting 2V8 mode");
    ESP_ERROR_CHECK(sensor_app_i2c_write(VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, SENSOR_APP_POWER_2V8));

    // Set the sensor's range mode
    ESP_LOGI(TAG, "Setting sensor's range mode");
    ESP_ERROR_CHECK(sensor_app_i2c_write(VL53L0X_REG_SYSRANGE_START, SENSOR_APP_SENSOR_MODE_CONTIUOUS));

    // Enable GPIO Interrupt when value is below threshold
    ESP_LOGI(TAG, "Enabling GPIO interrupt");
    ESP_ERROR_CHECK(sensor_app_i2c_write(VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, SENSOR_APP_GPIO_INTR_LEVEL_LOW));

    // Set GPIO output to LOW on interrupt
    ESP_LOGI(TAG, "Setting GPIO output");
    ESP_ERROR_CHECK(sensor_app_i2c_write(VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, SENSOR_APP_ACTIVE_STATE_LOW));

    // Set the threshold
    // ESP_LOGI(TAG, "Setting threshold");
    // ESP_ERROR_CHECK(sensor_app_i2c_write(VL53L0X_REG_SYSTEM_THRESH_LOW, SENSOR_OPEN_DISTANCE_MM));
}

void sensor_app_init() {
    ESP_LOGI(TAG, "Initializing sensor application");

    // Configure master bus
    sensor_app_i2c_configure_driver();

    // Check for available devices
    i2c_scanner();

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

esp_err_t sensor_app_check_connection(bool debugLogs) {
    uint8_t model_id;
    const esp_err_t err = sensor_app_receive_data(VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id, sizeof(model_id), debugLogs);
    if (!debugLogs) return err;
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Sensor is connected! Received model ID: %d", model_id);
    } else {
        ESP_LOGE(TAG, "I2C write failed with error: %s", esp_err_to_name(err));
    }
    return err;
}

static esp_err_t sensor_app_i2c_write(const uint8_t reg_addr, const uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(cmd);
    if (err != ESP_OK) return err;

    // Write address
    ESP_LOGI(TAG, "Writing sensor's address");
    err = i2c_master_write_byte(cmd, (SENSOR_I2C_WRITE_ADDR << 1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) return err;

    // Write register
    ESP_LOGI(TAG, "Writing sensor's register");
    err = i2c_master_write_byte(cmd, reg_addr, true);
    if (err != ESP_OK) return err;

    // Write data
    ESP_LOGI(TAG, "Writing sensor's data");
    err = i2c_master_write_byte(cmd, data, true);
    if (err != ESP_OK) return err;

    // Write stop bit
    ESP_LOGI(TAG, "Writing stop bit");
    err = i2c_master_stop(cmd);
    if (err != ESP_OK) return err;

    // Transmit data
    ESP_LOGI(TAG, "Transmitting data");
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
    if (err != ESP_OK) return err;

    // Clear used resources
    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

esp_err_t sensor_app_receive_data(const uint8_t reg_addr, uint8_t *data, size_t len, bool debugLogs) {
    if (debugLogs) ESP_LOGI(TAG, "Recieving data from sensor...");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(cmd);
    if (err != ESP_OK) return err;

    // 1. Write to sensor's register

    // Write address
    if (debugLogs) ESP_LOGI(TAG, "Writing address for writing");
    err = i2c_master_write_byte(cmd, (SENSOR_I2C_WRITE_ADDR << 1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) return err;

    // Write register
    if (debugLogs) ESP_LOGI(TAG, "Writing register");
    err = i2c_master_write_byte(cmd, reg_addr, true);
    if (err != ESP_OK) return err;

    // Write stop bit
    if (debugLogs) ESP_LOGI(TAG, "Writing stop bit");
    err = i2c_master_stop(cmd);
    if (err != ESP_OK) return err;

    // Transmit data
    if (debugLogs) ESP_LOGI(TAG, "CMD begin");
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
    if (err != ESP_OK) return err;

    // 2. Read data from sensor
    cmd = i2c_cmd_link_create();
    err = i2c_master_start(cmd);
    if (err != ESP_OK) return err;

    // Read address
    if (debugLogs) ESP_LOGI(TAG, "Writing address for reading");
    err = i2c_master_write_byte(cmd, (SENSOR_I2C_READ_ADDR << 1) | I2C_MASTER_READ, true);
    if (err != ESP_OK) return err;

    // Read data
    if (debugLogs) ESP_LOGI(TAG, "Reading data");
    err = i2c_master_read(cmd, data, len, true);
    if (err != ESP_OK) return err;

    // Write stop bit
    if (debugLogs) ESP_LOGI(TAG, "Writing stop bit");
    err = i2c_master_stop(cmd);
    if (err != ESP_OK) return err;

    // Recieve data
    if (debugLogs) ESP_LOGI(TAG, "CMD begin");
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
    if (err != ESP_OK) return err;

    // Clear used resources
    i2c_cmd_link_delete(cmd);

    return ESP_OK;
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
