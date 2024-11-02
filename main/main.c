//
// Created by kok on 08.10.24.
//

#include <power_app/power_app.h>
#include <pwm_app/pwm_app.h>
#include <sensor_app/sensor_app.h>

void app_main() {
    pwm_init();
    sensor_app_init();
    // power_app_init();
}
