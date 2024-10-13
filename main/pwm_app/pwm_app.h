//
// Created by kok on 11.10.24.
//

#ifndef PWM_APP_H
#define PWM_APP_H

#define PWM_APP_GROUP_ID              1
#define PWM_APP_RESOLUTION_HZ         1000000
#define PWM_APP_FREQ_HZ               50
#define PWM_APP_GENERATOR_GPIO        5
#define PWM_APP_MIN_PULSEWIDTH_US     500 // 500 microseconds
#define PWM_APP_MAX_PULSEWIDTH_US     2500 // 2500 microseconds

#define PWM_APP_MIN_SERVO_DEGREES     -90
#define PWM_APP_MAX_SERVO_DEGREES     90
#define PWM_APP_SERVO_OPEN_DEGREES    90
#define PWM_APP_SERVO_CLOSE_DEGREES   0

#include <freertos/FreeRTOS.h>

typedef enum {
    PWM_APP_MSG_OPEN_SERVO,
    PWM_APP_MSG_CLOSE_SERVO
} pwm_app_msg_e;

typedef struct {
    pwm_app_msg_e msgID;
} pwm_app_message_t;

void pwm_init();

void pwm_app_send_message(pwm_app_msg_e msgID);

bool pwm_app_get_servo_open();

#endif //PWM_APP_H
