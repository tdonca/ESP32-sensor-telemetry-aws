#### Steps to run:
- Connect SDA to GPIO defined in the code
- Connect SCL to GPIO defined in the code
- `$> idf.py build`
- `$> idf.py -p /dev/ttyUSB1 flash`
- `$> idf.py -p /dev/ttyUSB1 monitor`

#### Example output:
```
I (326) sensor-measure-i2c: STARTING SENSOR READING APPLICATION
I (336) sensor-measure-i2c: CONFIGURED I2C PARAMS
I (336) sensor-measure-i2c: INSTALLED I2C DRIVER
I (3346) sensor-measure-i2c: STARTED SENSOR READING LOOP
I (3396) sensor-measure-i2c: Raw data: 24660, Temperature Reading: 19.270227 C


I (6396) sensor-measure-i2c: STARTED SENSOR READING LOOP
I (6446) sensor-measure-i2c: Raw data: 24628, Temperature Reading: 19.184426 C
```