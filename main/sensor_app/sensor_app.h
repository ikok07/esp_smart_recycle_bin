//
// Created by kok on 12.10.24.
//

#ifndef SENSOR_APP_H
#define SENSOR_APP_H

#define SENSOR_SDA_GPIO                                                 (21)
#define SENSOR_SCL_GPIO                                                 (22)
#define SENSOR_INTR_GPIO                                                (23)
#define SENSOR_I2C_WRITE_ADDR                                           (0x29)
#define SENSOR_I2C_READ_ADDR                                            (0x29)
#define SENSOR_i2C_SCL_FREQ_HZ                                          (100000)
#define SENSOR_OPEN_DISTANCE_MM                                         (250)

void sensor_app_init();

#endif //SENSOR_APP_H
