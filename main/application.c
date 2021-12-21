#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "temp_sensor.h"


static const char *TAG = "application-main";

/***************************************/

int app_main(void) {
    ESP_LOGI(TAG, "STARTING SENSOR READING APPLICATION");
    ESP_ERROR_CHECK(sensor_init());

    // Loop for measuring temperature
    while (1) {

        // Delay between readings
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "STARTED SENSOR READING LOOP");
        
        // Get temperature reading from the sensor
        double temperature = 0;
        if (ESP_OK == sensor_read_temperature(&temperature)) {
            ESP_LOGI(TAG, "Temperature Reading: %.2f C\n", temperature);
        }        
    }  
}