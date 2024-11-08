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

static const char TAG[] = "sensor_app";
static vl53l0x_t *sensor_driver = NULL;
static SemaphoreHandle_t sensor_isr_semaphone;

static void i2c_scanner();

static void sensor_app_task() {
    esp_err_t err;
    while (1) {
        // const uint16_t distance = vl53l0x_readRangeSingleMillimeters(sensor_driver);
        // ESP_LOGI(TAG, "Distance: %d", distance);
        // if (distance < SENSOR_OPEN_DISTANCE_MM) pwm_app_send_message(PWM_APP_MSG_OPEN_SERVO);
        // else pwm_app_send_message(PWM_APP_MSG_CLOSE_SERVO);
        if (xSemaphoreTake(sensor_isr_semaphone, portMAX_DELAY)) {
            ESP_LOGI(TAG, "INTERRUPT TRIGGERED");
        }
        vTaskDelay(10);
    }
}

void isr_handler(void *arg) {
    xSemaphoreGive(sensor_isr_semaphone);
}

void sensor_app_init() {
    ESP_LOGI(TAG, "Initializing sensor application");
    sensor_driver = vl53l0x_config(I2C_NUM_0, SENSOR_SCL_GPIO, SENSOR_SDA_GPIO, -1, SENSOR_I2C_WRITE_ADDR, true);
    if (sensor_driver == NULL) {
        ESP_LOGE(TAG, "Failed to configure sensor's driver! The device is not connected!");
        esp_restart();
    }
    // i2c_scanner();
    const char *err_str = vl53l0x_init(sensor_driver);
    if (err_str) {
        ESP_LOGE(TAG, "Failed initialize sensor's driver! Error: %s", err_str);
        while(1) {
            vTaskDelay(10);
        };
        // esp_restart();
    }
    vl53l0x_startContinuous(sensor_driver, 0);

    ESP_ERROR_CHECK(gpio_set_direction(SENSOR_INTR_GPIO, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_intr_type(SENSOR_INTR_GPIO, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_set_pull_mode(SENSOR_INTR_GPIO, GPIO_PULLUP_ONLY));

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(SENSOR_INTR_GPIO, isr_handler, NULL);

    sensor_isr_semaphone = xSemaphoreCreateBinary();

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

void i2c_scanner() {
    i2c_cmd_handle_t cmd;
    esp_err_t err;
    ESP_LOGI(TAG, "Scanning...");
    for (int i = 1; i < 127; i++) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Found device at 0x%02x", i);
        }
    }
    printf("Scan complete.\n");
}