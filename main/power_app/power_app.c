//
// Created by kok on 23.10.24.
//

#include "power_app.h"

#include <esp_log.h>
#include <esp_sleep.h>
#include <tasks.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/task.h>

static const char TAG[] = "power_app";

static void power_app_wait_for_sleep_signal() {
    ESP_LOGI(TAG, "Waiting for the sensor's interrupt GPIO to go HIGH...");
    while(gpio_get_level(POWER_APP_SENSOR_INTR_GPIO) == 0) {
        vTaskDelay(10);
    }
}

static void power_app_configure_wakeup_source() {
    ESP_LOGI(TAG, "Configuring wakeup source");
    const gpio_config_t gpio_configuration = {
        .intr_type = GPIO_INTR_LOW_LEVEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = false,
        .pull_down_en = true,
        .pin_bit_mask = BIT64(POWER_APP_SENSOR_INTR_GPIO)
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_configuration));
    ESP_ERROR_CHECK(gpio_wakeup_enable(POWER_APP_SENSOR_INTR_GPIO, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());

    power_app_wait_for_sleep_signal();
    ESP_LOGI(TAG, "Wakeup source configured successfully!");
}

static void power_app_task() {
    while (1) {
        ESP_LOGI(TAG, "Entering light-sleep mode");

        // Make sure the log message is transmitted
        uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);

        esp_light_sleep_start();

        power_app_wait_for_sleep_signal();
    }
}

void power_app_init() {
    power_app_configure_wakeup_source();

    xTaskCreatePinnedToCore(
        power_app_task,
        "power_app_task",
        POWER_APP_TASK_STACK_SIZE,
        NULL,
        POWER_APP_TASK_PRIORITY,
        NULL,
        POWER_APP_TASK_CORE_ID
    );
}
