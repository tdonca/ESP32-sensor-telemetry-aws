#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "sensor-measure-i2c";

//TODO: why do so many GPIO pins not work? (18, 19, 27 work so far...)
#define I2C_MASTER_SCL_GPIO 27
#define I2C_MASTER_SDA_GPIO 18
#define I2C_CONTROLLER_NUM 0
#define I2C_MASTER_CLOCK_SPEED 400000
#define I2C_MASTER_TIMEOUT 1000

#define SENSOR_HTU21D_ADDR 0x40
#define SENSOR_TEMP_READ_COMMAND_NH 0xF3

//TODO: break up main() into helper funcs
int app_main(void) {
    ESP_LOGI(TAG, "STARTING SENSOR READING APPLICATION");

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
    ESP_ERROR_CHECK(i2c_driver_install(I2C_CONTROLLER_NUM, config.mode, 0, 0, 0));
    ESP_LOGI(TAG, "INSTALLED I2C DRIVER");

    // Loop for measuring temperature
    while (1) {

        // Delay between readings
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "STARTED SENSOR READING LOOP");
        
        // Command link to request a temp reading from the sensor
        i2c_cmd_handle_t cmd_request = i2c_cmd_link_create();
        i2c_master_start(cmd_request);
        i2c_master_write_byte(cmd_request, (SENSOR_HTU21D_ADDR << 1 | I2C_MASTER_WRITE), true);
        i2c_master_write_byte(cmd_request, SENSOR_TEMP_READ_COMMAND_NH, true);
        i2c_master_stop(cmd_request);
        esp_err_t result = i2c_master_cmd_begin(I2C_CONTROLLER_NUM, cmd_request, I2C_MASTER_TIMEOUT / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd_request);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, 
                    "I2C Initial Request master_cmd_begin failed: %s. Skipping iteration...", 
                    esp_err_to_name(result));
            continue;
        }

        // Delay for sensor measurement (max 50ms)
        vTaskDelay(50 / portTICK_PERIOD_MS);

        // Command link to read the temp data from the sensor
        uint8_t sensor_data[2] = {0};
        uint8_t crc = 0;
        i2c_cmd_handle_t cmd_read = i2c_cmd_link_create();
        i2c_master_start(cmd_read);
        i2c_master_write_byte(cmd_read, (SENSOR_HTU21D_ADDR << 1 | I2C_MASTER_READ), true);
        i2c_master_read(cmd_read, sensor_data, 2, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd_read, &crc, I2C_MASTER_NACK);
        i2c_master_stop(cmd_read);
        result = i2c_master_cmd_begin(I2C_CONTROLLER_NUM, cmd_read, I2C_MASTER_TIMEOUT / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd_read);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, 
                    "I2C Read Sensor Measurement command failed: %s. Skipping iteration...",
                    esp_err_to_name(result));
            continue;
        }

        // CRC check
        //TODO: implement

        // Raw data conversion
        uint16_t raw_temp = sensor_data[0];
        raw_temp = (raw_temp << 8); // move MSB into bits 15-8
        raw_temp = raw_temp | (uint16_t)sensor_data[1]; // add LSB into bits 7-0
        raw_temp = raw_temp & 0xFFFC; // remove the last two status bits
        double temperature = (raw_temp * 175.72 / 65536.0) - 46.85;

        // Print temp reading to monitor
        ESP_LOGI(TAG, "Raw data: %d, Temperature Reading: %f C\n\n", raw_temp, temperature);
    }  
}