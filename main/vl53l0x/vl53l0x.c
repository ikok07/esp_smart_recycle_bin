//
// Created by Kok on 11/4/24.
//

#include "vl53l0x.h"

#include <esp_log.h>
#include <driver/i2c.h>

static const char TAG[] = "vl53l0x";

static uint8_t sensor_addr = 0;
static uint8_t stop_variable = 0;

static esp_err_t vl53l0x_configure_driver(vl53l0x_init_config_t config);
static esp_err_t vl53l0x_data_init();
static esp_err_t vl53l0x_static_init();
static esp_err_t vl53l0x_load_default_tuning_settings();
static esp_err_t vl53l0x_configure_interrupt();
static esp_err_t vl53l0x_set_sequence_steps_enabled(uint8_t sequence_step);
static esp_err_t vl53l0x_perform_ref_calibration();
static esp_err_t vl53l0x_perform_single_ref_calibration(vl53l0x_calibration_type_t calib_type);
static esp_err_t vl53l0x_i2c_write(uint8_t reg_addr, uint8_t data);
static esp_err_t vl53l0x_i2c_read(uint8_t reg_addr, uint8_t *data, size_t len, bool debugLogs);
static void i2c_scanner();

// // Set the sensor's range mode
// ESP_LOGI(TAG, "Setting sensor's range mode");
// ESP_ERROR_CHECK(sensor_app_i2c_write(VL53L0X_REG_SYSRANGE_START, SENSOR_APP_SENSOR_MODE_CONTIUOUS)); // old (0x00, 0x01)
//
// ESP_ERROR_CHECK(sensor_app_i2c_write(0xFF, 0x00));
// ESP_ERROR_CHECK(sensor_app_i2c_write(0x80, 0x00));

// Set the threshold
// ESP_LOGI(TAG, "Setting threshold");
// ESP_ERROR_CHECK(sensor_app_i2c_write(VL53L0X_REG_SYSTEM_THRESH_LOW, SENSOR_OPEN_DISTANCE_MM));

esp_err_t vl53l0x_init(const vl53l0x_init_config_t config) {
    esp_err_t err = vl53l0x_configure_driver(config);
    if (err != ESP_OK) return err;

    i2c_scanner();

    esp_err_t check_err;
    while ((check_err = vl53l0x_check_connection()) != ESP_OK) {
        ESP_LOGE(TAG, "Sensor is not connected! Waiting for connection... (Error: %s)", esp_err_to_name(check_err));
        vTaskDelay(10);
    }

    err = vl53l0x_data_init();
    if (err != ESP_OK) return err;

    err = vl53l0x_static_init();
    if (err != ESP_OK) return err;

    return vl53l0x_perform_ref_calibration();
}

esp_err_t vl53l0x_check_connection() {
    uint8_t model_id;
    const esp_err_t err = vl53l0x_i2c_read(VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id, sizeof(model_id), true);
    if (err == ESP_OK && model_id == VL53L0X_MODEL_ID) ESP_LOGI(TAG, "Sensor is connected!");
    else ESP_LOGE(TAG, "I2C write failed with error: %s", esp_err_to_name(err));
    return err;
}

esp_err_t vl53l0x_configure_driver(vl53l0x_init_config_t config) {
    ESP_LOGI(TAG, "Configuring I2C Driver");
    const i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config.sda_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = config.scl_gpio,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = config.i2c_freq_hz,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
    };
    esp_err_t err = i2c_param_config(I2C_NUM_0, &i2c_config);
    if (err != ESP_OK) return err;

    err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) return err;

    sensor_addr = config.i2c_addr;

    ESP_LOGI(TAG, "I2C driver successfully configured");
    return ESP_OK;
}

esp_err_t vl53l0x_data_init() {
    ESP_LOGI(TAG, "Setting 2V8 mode");
    esp_err_t err = vl53l0x_i2c_write(VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, VL53L0X_POWER_2V8);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Setting I2C to standard mode");
    err = vl53l0x_i2c_write(0x88, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x80, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x00, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_read(0x91, &stop_variable, sizeof(stop_variable), false); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x00, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x80, 0x00); if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t vl53l0x_static_init() {
    esp_err_t err;
    if ((err = vl53l0x_load_default_tuning_settings()) != ESP_OK) return err;
    if ((err = vl53l0x_configure_interrupt()) != ESP_OK) return err;
    return vl53l0x_set_sequence_steps_enabled(
        RANGE_SEQUENCE_STEP_DSS +
        RANGE_SEQUENCE_STEP_PRE_RANGE +
        RANGE_SEQUENCE_STEP_FINAL_RANGE);
}

esp_err_t vl53l0x_load_default_tuning_settings() {
    ESP_LOGI(TAG, "Loading the default tuning settings");
    esp_err_t err = vl53l0x_i2c_write(0xFF, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x00, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x09, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x10, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x11, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x24, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x25, 0xFF); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x75, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x4E, 0x2C); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x48, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x30, 0x20); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x30, 0x09); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x54, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x31, 0x04); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x32, 0x03); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x40, 0x83); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x46, 0x25); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x60, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x27, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x50, 0x06); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x51, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x52, 0x96); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x56, 0x08); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x57, 0x30); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x61, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x62, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x64, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x65, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x66, 0xA0); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x22, 0x32); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x47, 0x14); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x49, 0xFF); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x4A, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x7A, 0x0A); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x7B, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x78, 0x21); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x23, 0x34); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x42, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x44, 0xFF); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x45, 0x26); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x46, 0x05); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x40, 0x40); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x0E, 0x06); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x20, 0x1A); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x43, 0x40); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x34, 0x03); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x35, 0x44); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x31, 0x04); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x4B, 0x09); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x4C, 0x05); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x4D, 0x04); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x44, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x45, 0x20); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x47, 0x08); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x48, 0x28); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x67, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x70, 0x04); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x71, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x72, 0xFE); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x76, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x77, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x0D, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x00); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x80, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x01, 0xF8); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0xFF, 0x01); if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(0x8E, 0x01);
    return err;
}

esp_err_t vl53l0x_configure_interrupt() {
    ESP_LOGI(TAG, "Enabling GPIO interrupt");
    esp_err_t err = vl53l0x_i2c_write(VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, VL53L0X_GPIO_INTR_NEW_SAMPLE_READY);
    if (err != ESP_OK) return err;

    // Set GPIO output to LOW on interrupt
    ESP_LOGI(TAG, "Setting GPIO output");
    uint8_t gpio_hv_mux_active_high = 0;
    err = vl53l0x_i2c_read(VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, &gpio_hv_mux_active_high, sizeof(gpio_hv_mux_active_high), false);
    if (err != ESP_OK) return err;
    gpio_hv_mux_active_high &= ~0x10;
    err = vl53l0x_i2c_write(VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv_mux_active_high);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Clearing sensor's interrupt");
    err = vl53l0x_i2c_write(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    return err;
}

esp_err_t vl53l0x_set_sequence_steps_enabled(uint8_t sequence_step) {
    return vl53l0x_i2c_write(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, sequence_step);
}

/*
 * WARNING: Temperature calibration needs to be run again if the temperature changes by
 * more than 8 degrees according to the datasheet.
 */
esp_err_t vl53l0x_perform_ref_calibration() {
    esp_err_t err = vl53l0x_perform_single_ref_calibration(VL53L0X_CALIBRATION_TYPE_VHV);
    if (err != ESP_OK) return err;

    err = vl53l0x_perform_single_ref_calibration(VL53L0X_CALIBRATION_TYPE_PHASE);
    if (err != ESP_OK) return err;

    // Restore sequence steps enabled
    return vl53l0x_set_sequence_steps_enabled(RANGE_SEQUENCE_STEP_DSS +
                                    RANGE_SEQUENCE_STEP_PRE_RANGE +
                                    RANGE_SEQUENCE_STEP_FINAL_RANGE);
}

esp_err_t vl53l0x_perform_single_ref_calibration(vl53l0x_calibration_type_t calib_type) {
    uint8_t sysrange_start = 0;
    uint8_t sequence_config = 0;
    switch (calib_type) {
        case VL53L0X_CALIBRATION_TYPE_VHV:
            sequence_config = 0x01;
            sysrange_start = 0x01 | 0x40;
        break;
        case VL53L0X_CALIBRATION_TYPE_PHASE:
            sequence_config = 0x02;
            sysrange_start = 0x01 | 0x00;
        break;
    }

    esp_err_t err = vl53l0x_i2c_write(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, sequence_config);
    if (err != ESP_OK) return err;
    err = vl53l0x_i2c_write(VL53L0X_REG_SYSRANGE_START, sysrange_start);
    if (err != ESP_OK) return err;

    // Wait for interrupt
    uint8_t interrupt_status = 0;
    do {
        err = vl53l0x_i2c_read(VL53L0X_REG_RESULT_INTERRUPT_STATUS, &interrupt_status, sizeof(interrupt_status), false);
    } while (err == ESP_OK && ((interrupt_status & 0x07) == 0));
    if (err != ESP_OK) return err;

    err = vl53l0x_i2c_write(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (err != ESP_OK) return err;

    err = vl53l0x_i2c_write(VL53L0X_REG_SYSRANGE_START, 0x00);
    return err;
}

esp_err_t vl53l0x_i2c_write(const uint8_t reg_addr, const uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(cmd);
    if (err != ESP_OK) return err;

    // Write address
    ESP_LOGI(TAG, "Writing sensor's address");
    err = i2c_master_write_byte(cmd, (sensor_addr << 1) | I2C_MASTER_WRITE, true);
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

esp_err_t vl53l0x_i2c_read(const uint8_t reg_addr, uint8_t *data, size_t len, bool debugLogs) {
    if (debugLogs) ESP_LOGI(TAG, "Recieving data from sensor...");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(cmd);
    if (err != ESP_OK) return err;

    // 1. Write to sensor's register

    // Write address
    if (debugLogs) ESP_LOGI(TAG, "Writing address for writing");
    err = i2c_master_write_byte(cmd, (sensor_addr << 1) | I2C_MASTER_WRITE, true);
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
    err = i2c_master_write_byte(cmd, (sensor_addr << 1) | I2C_MASTER_READ, true);
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
    ESP_LOGI(TAG, "Clearing I2C CMD");
    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

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