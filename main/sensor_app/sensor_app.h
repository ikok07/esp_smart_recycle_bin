//
// Created by kok on 12.10.24.
//

#ifndef SENSOR_APP_H
#define SENSOR_APP_H

#define SENSOR_GPIO                 16

#define SENSOR_SDA_GPIO             18
#define SENSOR_SCL_GPIO             19
#define SENSOR_I2C_ADDR             0x52
#define SENSOR_i2C_SCL_FREQ_HZ      100000

#include <esp_err.h>

void sensor_app_init();

esp_err_t sensor_app_check_connection();

esp_err_t sensor_app_receive_data();

#endif //SENSOR_APP_H
