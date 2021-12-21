#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"

#include "temp_sensor.h"

static const char *TAG = "sensor-measure-i2c";

#define I2C_CONTROLLER_NUM 0
#define I2C_MASTER_CLOCK_SPEED 400000
#define I2C_MASTER_TIMEOUT 1000

#define SENSOR_HTU21D_ADDR 0x40
#define SENSOR_TEMP_READ_COMMAND_NH 0xF3
#define SENSOR_HUMID_READ_COMMAND_NH 0xF5
#define SENSOR_SOFT_RESET_COMMAND 0xFE

#define SENSOR_DELAY_RESET_MS 15
#define SENSOR_DELAY_TEMP_MS 50

/********************************************/

static double compute_temperature(uint8_t msb, uint8_t lsb) {
    uint16_t raw_temp = msb;
    raw_temp = (raw_temp << 8); // move MSB into bits 15-8
    raw_temp = raw_temp | (uint16_t)lsb; // add LSB into bits 7-0
    raw_temp = raw_temp & 0xFFFC; // remove the last two status bits
    ESP_LOGD(TAG, "Raw temp. data: 0x%x", raw_temp);

    return (raw_temp * 175.72 / 65536.0) - 46.85;
}

static uint8_t compute_crc8(uint8_t msb, uint8_t lsb) {
    uint32_t data_row = ((uint32_t)msb << 16) | ((uint32_t)lsb << 8); // [msb_lsb_00000000] 24bit
    uint32_t divisor_poly = 0x131; // [100110001] 9-bit CRC8 polynomial X^8+X^5+X^4+1
    divisor_poly <<= 15; // align 9bit divisor with the left side of 24bit row

    // Shift the divisor along the data row until the divisor right-edge 
    // reaches the data row right-edge. This operation "divides" the data 
    // row by the divisor. Leaving the last 8 bits as the 
    // "division remainder". This calculated remainder is the CRC value.
    for (int i = 0; i < 16; i++) {
        
        // Perform XOR if the left-most 1bit of the divisor aligns with a 1bit in the data row
        // The initial left-most 1bit of the 24bit aligned divisor is hardcoded as 23 
        // The offset calculation tracks this index as the divisor shifts along the data row
        if ((data_row >> (23 - i)) == 1) {
            data_row = data_row ^ divisor_poly;
        }

        divisor_poly >>= 1;
    }

    // Return the remaining 8 least significant bits as the computed CRC value
    return (data_row & 0xFF);
}

static esp_err_t sensor_write_command_i2c(int command) {
    i2c_cmd_handle_t cmd_write = i2c_cmd_link_create();
    i2c_master_start(cmd_write);
    i2c_master_write_byte(cmd_write, (SENSOR_HTU21D_ADDR << 1 | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd_write, command, true);
    i2c_master_stop(cmd_write);
    esp_err_t result = i2c_master_cmd_begin(I2C_CONTROLLER_NUM, cmd_write, I2C_MASTER_TIMEOUT / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_write);
    
    return result;
}

static esp_err_t sensor_read_command_i2c(uint8_t *data_msb, uint8_t *data_lsb) {
    uint8_t data_crc = 0;
    i2c_cmd_handle_t cmd_read = i2c_cmd_link_create();
    i2c_master_start(cmd_read);
    i2c_master_write_byte(cmd_read, (SENSOR_HTU21D_ADDR << 1 | I2C_MASTER_READ), true);
    i2c_master_read_byte(cmd_read, data_msb, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd_read, data_lsb, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd_read, &data_crc, I2C_MASTER_NACK);
    i2c_master_stop(cmd_read);
    esp_err_t result = i2c_master_cmd_begin(I2C_CONTROLLER_NUM, cmd_read, I2C_MASTER_TIMEOUT / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_read);

    // Perform CRC on the sensor data
    if (result == ESP_OK) {
        uint8_t computed_crc = compute_crc8(*data_msb, *data_lsb);
        if (data_crc != computed_crc) {
            ESP_LOGE(TAG, "Sensor data CRC failed due to mismatch.");
            ESP_LOGD(TAG, "Calculated CRC: 0x%x sensor provided CRC: 0x%x", computed_crc, data_crc);
            result = ESP_FAIL;            
        }
    }

    return result;
}

/********************************************/

esp_err_t sensor_init(void) {
    
    // Configure the I2C Driver
    i2c_config_t config = {};
    config.mode = I2C_MODE_MASTER;
    config.scl_io_num = I2C_MASTER_SCL_GPIO;
    config.sda_io_num = I2C_MASTER_SDA_GPIO;
    config.scl_pullup_en = false;
    config.sda_pullup_en = false;
    config.master.clk_speed = I2C_MASTER_CLOCK_SPEED;
    i2c_param_config(I2C_CONTROLLER_NUM, &config);
    ESP_LOGI(TAG, "CONFIGURED I2C PARAMS");

    // Install the I2C driver
    esp_err_t result = i2c_driver_install(I2C_CONTROLLER_NUM, config.mode, 0, 0, 0);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "I2C Driver install failed. Reason: %s", esp_err_to_name(result));
        return result;
    }
    ESP_LOGI(TAG, "INSTALLED I2C DRIVER");

    // Reset the sensor and verify operation
    //TODO: prevent infinite loop if the sensor is broken
    ESP_LOGI(TAG, "PERFORMING SOFT RESET AND WAITING FOR SENSOR RESPONSE...");
    do {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        result = sensor_write_command_i2c(SENSOR_SOFT_RESET_COMMAND);
        vTaskDelay(SENSOR_DELAY_RESET_MS / portTICK_PERIOD_MS);
    } while(result != ESP_OK);

    return result;
}

esp_err_t sensor_read_temperature(double* result_temp) {
    if (result_temp == NULL) {
        return ESP_FAIL;
    }

    // Send a temperature reading request to the sensor
    esp_err_t result = sensor_write_command_i2c(SENSOR_TEMP_READ_COMMAND_NH);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Temperature reading I2C write command failed. Reason: %s", esp_err_to_name(result));
        return result;
    }

    // Wait for sensor measurement
    vTaskDelay(SENSOR_DELAY_TEMP_MS / portTICK_PERIOD_MS);

    // Read the data from the sensor
    uint8_t data_buf[2] = {0};
    result = sensor_read_command_i2c(&data_buf[0], &data_buf[1]);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Temperature reading I2C read command failed. Reason: %s", esp_err_to_name(result));
        return result;
    }

    // Convert the raw data to a temperature value
    *result_temp = compute_temperature(data_buf[0], data_buf[1]);

    return ESP_OK;
}
