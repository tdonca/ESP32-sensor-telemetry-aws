#include "esp_err.h"

// Public interface header for the temperature sensor API

// The GPIO pins used for I2C communication with the sensor
#define I2C_MASTER_SCL_GPIO 27
#define I2C_MASTER_SDA_GPIO 18

// Initializes the temperature sensor and verifies that it
// is ready for communication.
esp_err_t sensor_init(void);

// Reads the current value from the temperature sensor. The
// value is returned in result_temp in Celsius units.
esp_err_t sensor_read_temperature(double* result_temp);