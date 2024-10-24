#include "vl53l0x_platform.h"

// #include <driver/i2c.h>

#include <portmacro.h>
#include <driver/i2c_master.h>

#define ACK_CHECK_EN true

VL53L0X_Error esp_to_vl53l0x_error(esp_err_t esp_err) {
    switch (esp_err) {
        case ESP_OK:
            return VL53L0X_ERROR_NONE;
        case ESP_ERR_INVALID_ARG:
            return VL53L0X_ERROR_INVALID_PARAMS;
        case ESP_FAIL:
        case ESP_ERR_INVALID_STATE:
            return VL53L0X_ERROR_CONTROL_INTERFACE;
        case ESP_ERR_TIMEOUT:
            return VL53L0X_ERROR_TIME_OUT;
        default:
            return VL53L0X_ERROR_UNDEFINED;
    }
}

esp_err_t VL53L0X_I2C_INIT(VL53L0X_DEV Dev, i2c_master_bus_handle_t *sensor_app_i2c_bus_handle, i2c_master_dev_handle_t *sensor_app_i2c_device_handle) {
    esp_err_t esp_err;
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = Dev->sda_gpio,
        .scl_io_num = Dev->scl_gpio,
        .clk_source = Dev->clk_src,
        .glitch_ignore_cnt = Dev->glitch_ignore_cnt, // increase if data is unstable
        .flags = {.enable_internal_pullup = Dev->enable_internal_pullup} // Pull up should be from external resistor
    };
    esp_err = i2c_new_master_bus(&bus_config, &sensor_app_i2c_bus_handle);
    if (esp_err != ESP_OK) return esp_err;

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = Dev->i2c_address,
        .scl_speed_hz = Dev->scl_speed_hz,
        .scl_wait_us = Dev->scl_wait_us,
        .flags = {
            .disable_ack_check = false
        }
    };
    esp_err = i2c_master_bus_add_device(sensor_app_i2c_bus_handle, &device_config, &sensor_app_i2c_device_handle);
    return esp_err;
}

/**
 * Writes the supplied byte buffer to the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to uint8_t buffer containing the data to be written
 * @param   count     Number of bytes in the supplied byte buffer
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    esp_err_t esp_err;
    i2c_master_bus_handle_t sensor_app_i2c_bus_handle;
    i2c_master_dev_handle_t sensor_app_i2c_device_handle;
    VL53L0X_I2C_INIT(Dev, &sensor_app_i2c_bus_handle, &sensor_app_i2c_device_handle);

    // write I2C address
    // i2c_master_write_byte(cmd, ( Dev->i2c_address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    // const uint8_t address = ( Dev->i2c_address << 1 ) | I2C_MASTER_WRITE;
    // esp_err = i2c_master_transmit(sensor_app_i2c_device_handle, &address, sizeof(address), portMAX_DELAY);
    // if (esp_err != ESP_OK) return esp_to_vl53l0x_error(esp_err);

    // write register
    // i2c_master_write_byte(cmd, index, ACK_CHECK_EN);
    esp_err = i2c_master_transmit(sensor_app_i2c_device_handle, &index, sizeof(index), portMAX_DELAY);
    if (esp_err != ESP_OK) return esp_to_vl53l0x_error(esp_err);

    // Data
    // Note: Needed to use i2c_master_write_byte as i2c_master_write will not expect an ack
    // after each byte
    // for (int i = 0; i < count; i++)
    // {
    //     // i2c_master_write_byte(cmd, *(pdata + i), ACK_CHECK_EN);
    //     uint8_t data = *(pdata + i);
    //     esp_err = i2c_master_transmit(sensor_app_i2c_device_handle, &data, sizeof(data), portMAX_DELAY);
    //     if (esp_err != ESP_OK) return esp_to_vl53l0x_error(esp_err);
    // }

    esp_err = i2c_master_transmit(sensor_app_i2c_device_handle, pdata, sizeof(uint8_t) * count, portMAX_DELAY);
    if (esp_err != ESP_OK) return esp_to_vl53l0x_error(esp_err);

    // i2c_master_stop(cmd);
    // esp_err_t ret = i2c_master_cmd_begin(Dev->i2c_port_num, cmd, 1000 / portTICK_PERIOD_MS);
    // i2c_cmd_link_delete(cmd);

    return esp_to_vl53l0x_error(ESP_OK);
}

/**
 * Reads the requested number of bytes from the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to the uint8_t buffer to store read data
 * @param   count     Number of uint8_t's to read
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    // I2C write
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t esp_err;
    i2c_master_bus_handle_t sensor_app_i2c_bus_handle;
    i2c_master_dev_handle_t sensor_app_i2c_device_handle;
    VL53L0X_I2C_INIT(Dev, &sensor_app_i2c_bus_handle, &sensor_app_i2c_device_handle);

    ////// First tell the VL53L0X which register we are reading from
    // ESP_ERROR_CHECK(i2c_master_start(cmd));

    // Write I2C address
    // ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ( Dev->i2c_address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN));
    // const uint8_t data = ( Dev->i2c_address << 1 ) | I2C_MASTER_WRITE;
    // i2c_master_transmit(sensor_app_i2c_device_handle, &data, sizeof(data), portMAX_DELAY);
    // Write register
    // ESP_ERROR_CHECK(i2c_master_write_byte(cmd, index, ACK_CHECK_EN));
    esp_err = i2c_master_transmit(sensor_app_i2c_device_handle, &index, sizeof(index), portMAX_DELAY);
    if (esp_err != ESP_OK) return esp_to_vl53l0x_error(esp_err);

    ////// Second, read from the register
    // ESP_ERROR_CHECK(i2c_master_start(cmd));

    // Write I2C address
    // ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ( Dev->i2c_address << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN));

    // Read data from register
    // ESP_ERROR_CHECK(i2c_master_read(cmd, pdata, count, I2C_MASTER_LAST_NACK));
    esp_err = i2c_master_receive(sensor_app_i2c_device_handle, pdata, sizeof(uint8_t) * count, portMAX_DELAY);
    if (esp_err != ESP_OK) return esp_to_vl53l0x_error(esp_err);

    // ESP_ERROR_CHECK(i2c_master_stop(cmd));
    // esp_err_t ret = i2c_master_cmd_begin(Dev->i2c_port_num, cmd, 1000 / portTICK_PERIOD_MS);
    // i2c_cmd_link_delete(cmd);

    return esp_to_vl53l0x_error(esp_err);
}

/**
 * Write single byte register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      8 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

/**
 * Write word register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      16 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    uint8_t buffer[2]; // 2
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data &  0x00FF);

    return VL53L0X_WriteMulti(Dev, index, buffer, 2);
}

/**
 * Write double word (4 byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      32 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buffer[4]; // 4

    buffer[0] = (uint8_t) (data >> 24);
    buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
    buffer[3] = (uint8_t) (data &  0x000000FF);

    return VL53L0X_WriteMulti(Dev, index, buffer, 4);
}

/**
 * Read single byte register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 8 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}

/**
 * Read word (2byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 16 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    VL53L0X_Error status;
    uint8_t  buffer[2];

    status = VL53L0X_ReadMulti(Dev, index, buffer, 2);
    *data = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];

    return status;
}

/**
 * Read dword (4byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 32 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    VL53L0X_Error status;
    uint8_t  buffer[4];

    status = VL53L0X_ReadMulti(Dev, index, buffer, 4);
    *data = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) +
             ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

    return status;
}

/**
 * Threat safe Update (read/modify/write) single byte register
 *
 * Final_reg = (Initial_reg & and_data) |or_data
 *
 * @param   Dev        Device Handle
 * @param   index      The register index
 * @param   AndData    8 bit and data
 * @param   OrData     8 bit or data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    VL53L0X_Error status;
    uint8_t data;

    status = VL53L0X_RdByte(Dev, index, &data);

    if (status != VL53L0X_ERROR_NONE)
        return status;

    data = (data & AndData) | OrData;

    return VL53L0X_WrByte(Dev, index, data);
}

/** @} end of VL53L0X_registerAccess_group */


/**
 * @brief execute delay in all polling API call
 *
 * A typical multi-thread or RTOs implementation is to sleep the task for some 5ms (with 100Hz max rate faster polling is not needed)
 * if nothing specific is need you can define it as an empty/void macro
 * @code
 * #define VL53L0X_PollingDelay(...) (void)0
 * @endcode
 * @param Dev       Device Handle
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    // Doesn't seem to be needed?
    //vTaskDelay(5 / portTICK_RATE_MS);
    return VL53L0X_ERROR_NONE;
}
