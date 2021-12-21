#### Steps to run:
- Connect SDA to GPIO defined in the code
- Connect SCL to GPIO defined in the code
- `$> idf.py build`
- `$> idf.py -p /dev/ttyUSB1 flash`
- `$> idf.py -p /dev/ttyUSB1 monitor`

#### Example output:
```
I (317) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (326) application-main: STARTING SENSOR READING APPLICATION
I (336) sensor-measure-i2c: CONFIGURED I2C PARAMS
I (336) sensor-measure-i2c: INSTALLED I2C DRIVER
I (346) sensor-measure-i2c: PERFORMING SOFT RESET AND WAITING FOR SENSOR RESPONSE...
I (5376) application-main: STARTED SENSOR READING LOOP
I (5426) application-main: Temperature Reading: 18.53 C

I (8426) application-main: STARTED SENSOR READING LOOP
I (8476) application-main: Temperature Reading: 18.51 C

...

```