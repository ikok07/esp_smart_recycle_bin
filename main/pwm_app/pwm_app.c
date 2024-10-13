//
// Created by kok on 11.10.24.
//

#include "pwm_app.h"

#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <driver/mcpwm_cmpr.h>
#include <driver/mcpwm_gen.h>
#include <driver/mcpwm_oper.h>
#include <driver/mcpwm_timer.h>

#include "tasks.h"

static const char TAG[] = "pwm_app";

mcpwm_timer_handle_t mcpwm_timer;
mcpwm_oper_handle_t mcpwm_operator;
mcpwm_cmpr_handle_t mcpwm_comparator;
mcpwm_gen_handle_t mcpwm_generator;

static QueueHandle_t pwm_app_msg_queue;
static EventGroupHandle_t pwm_app_event_group;

static uint8_t PWM_APP_SERVO_POS_OPEN       = BIT0;

static void pwm_app_set_servo_angle(int angle);

static void pwm_app_message_task(void *pvParams) {
    pwm_app_message_t msg;
    EventBits_t event_bits;

    while (1) {
        if (xQueueReceive(pwm_app_msg_queue, &msg, portMAX_DELAY)) {
            switch (msg.msgID) {
                case PWM_APP_MSG_OPEN_SERVO:
                    xEventGroupSetBits(pwm_app_event_group, PWM_APP_SERVO_POS_OPEN);
                    pwm_app_set_servo_angle(PWM_APP_SERVO_OPEN_DEGREES);
                case PWM_APP_MSG_CLOSE_SERVO:
                    xEventGroupClearBits(pwm_app_event_group, PWM_APP_SERVO_POS_OPEN);
                    pwm_app_set_servo_angle(PWM_APP_SERVO_CLOSE_DEGREES);
                default:
                    ESP_LOGE(TAG, "Invalid PWM message sent!");
            }
        }
    }
}

static uint32_t angle_to_compare_value(const int angle) {
    return (angle - PWM_APP_MIN_SERVO_DEGREES) * (PWM_APP_MAX_PULSEWIDTH_US - PWM_APP_MIN_PULSEWIDTH_US) / (PWM_APP_MAX_SERVO_DEGREES - PWM_APP_MIN_SERVO_DEGREES) + PWM_APP_MIN_PULSEWIDTH_US;
}

static void pwm_app_setup_components() {
    ESP_LOGI(TAG, "Creating timer");
    // Create PWM timer
    const mcpwm_timer_config_t timer_config = {
        .group_id = PWM_APP_GROUP_ID,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_APP_RESOLUTION_HZ,
        .period_ticks = PWM_APP_RESOLUTION_HZ / PWM_APP_FREQ_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &mcpwm_timer));

    ESP_LOGI(TAG, "Creating operator");
    // Create PWM Operator
    const mcpwm_operator_config_t operator_config = {
        .group_id = PWM_APP_GROUP_ID,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &mcpwm_operator));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(mcpwm_operator, mcpwm_timer));

    ESP_LOGI(TAG, "Creating Comparator");
    // Create PWM Comparator
    const mcpwm_comparator_config_t comparator_config = {
        .flags = {
            .update_cmp_on_tez = true
        }
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(mcpwm_operator, &comparator_config, &mcpwm_comparator));

    ESP_LOGI(TAG, "Creating Generator");
    // Create PWM Generator
    const mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = PWM_APP_GENERATOR_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(mcpwm_operator, &generator_config, &mcpwm_generator));
}

static void pwm_app_set_servo_angle(const int angle) {
    const esp_err_t err = mcpwm_comparator_set_compare_value(mcpwm_comparator, angle_to_compare_value(angle));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set servo angle! Error: %s", esp_err_to_name(err));
    }
}

void pwm_init() {
    ESP_LOGI(TAG, "Initializing PWM Application");

    // Configure the PWM Application
    pwm_app_setup_components();

    // Set initial servo angle to 90 degrees
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_comparator, angle_to_compare_value(0)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(mcpwm_generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(mcpwm_generator,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwm_comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(mcpwm_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "PWM Application initialized");

    pwm_app_msg_queue = xQueueCreate(3, sizeof(pwm_app_message_t));
    pwm_app_event_group = xEventGroupCreate();

    xTaskCreatePinnedToCore(
        pwm_app_message_task,
        "pwm_app_message_task",
        PWM_APP_TASK_STACK_SIZE,
        NULL,
        PWM_APP_TASK_PRIORITY,
        NULL,
        PWM_APP_TASK_CORE_ID
    );
}

void pwm_app_send_message(const pwm_app_msg_e msgID) {
    const pwm_app_message_t msg = {.msgID = msgID};
    xQueueSend(pwm_app_msg_queue, &msg, portMAX_DELAY);
}

bool pwm_app_get_servo_open() {
    const EventBits_t event_bits = xEventGroupGetBits(pwm_app_event_group);
    return event_bits & PWM_APP_SERVO_POS_OPEN;
}
