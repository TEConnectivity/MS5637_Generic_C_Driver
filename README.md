# MS5637 Generic C Driver
Generic C driver for the [MS5637 sensor](http://www.te.com/usa-en/product-CAT-BLPS0037.html)

![ms5637](http://www.te.com/content/dam/te-com/catalog/part/CAT/BLP/S00/CAT-BLPS0037-t1.jpg/jcr:content/renditions/product-details.png)

The MS5637 sensor is a self-contained pressure and temperature sensor that is  fully calibrated during manufacture. The sensor can operate from 1.5V to 3.6V. The sensor module includes a high-linearity pressure sensor and an ultra-low power 24 bit ΔΣ ADC with internal factory-calibrated coefficients.

### Specifications
* Measures pressure from 300mbar to 1200mbar
*	Measures temperature from -40°C to 125°C
*	I2C communication
*	Fully calibrated
*	Fast response time
*	Very low power consumption


### Driver features
* Connection test
* Reset
* Aquisition resolution management
* Temperature and pressure measurement


**NB:** This driver is intended to provide an implementation example of the sensor communication protocol, in order to be usable you have to implement a proper I2C layer for your target platform.
