/**
 *
 * MS5637 Altimeter Driver
 *
 * Copyright (c) 2020 James F Dougherty <jafrado@gmail.com>
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 *
 */
#ifndef MS5637_H
#define MS5637_H

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

enum ms5637_resolution_osr {
	MS5637_RESOLUTION_OSR_256 = 0,
	MS5637_RESOLUTION_OSR_512,
	MS5637_RESOLUTION_OSR_1024,
	MS5637_RESOLUTION_OSR_2048,
	MS5637_RESOLUTION_OSR_4096,
	MS5637_RESOLUTION_OSR_8192
};

enum ms5637_status {
	MS5637_OK,
	MS5637_NO_I2C_ACK,
	MS5637_ENXIO,
	MS5637_CRC_ERROR
};

/* MS5637 device address */
#define MS5637_ADDR                                             0x76 /* 0b1110110 */

/* MS5637 device commands */
#define MS5637_RESET_COMMAND                                    0x1E
#define MS5637_START_PRESSURE_ADC_CONVERSION			0x40
#define MS5637_START_TEMPERATURE_ADC_CONVERSION			0x50
#define MS5637_READ_ADC						0x00
#define MS5637_CONVERSION_OSR_MASK				0x0F

#define MS5637_CONVERSION_TIME_OSR_256				1000
#define MS5637_CONVERSION_TIME_OSR_512				2000
#define MS5637_CONVERSION_TIME_OSR_1024				3000
#define MS5637_CONVERSION_TIME_OSR_2048				5000
#define MS5637_CONVERSION_TIME_OSR_4096				9000
#define MS5637_CONVERSION_TIME_OSR_8192				17000

/* MS5637 commands */
#define MS5637_PROM_ADDRESS_READ_ADDRESS_0			0xA0
#define MS5637_PROM_ADDRESS_READ_ADDRESS_1			0xA2
#define MS5637_PROM_ADDRESS_READ_ADDRESS_2			0xA4
#define MS5637_PROM_ADDRESS_READ_ADDRESS_3			0xA6
#define MS5637_PROM_ADDRESS_READ_ADDRESS_4			0xA8
#define MS5637_PROM_ADDRESS_READ_ADDRESS_5			0xAA
#define MS5637_PROM_ADDRESS_READ_ADDRESS_6			0xAC
#define MS5637_PROM_ADDRESS_READ_ADDRESS_7			0xAE

/* Coefficients indexes for temperature and pressure computation */
#define MS5637_CRC_INDEX					0
#define MS5637_PRESSURE_SENSITIVITY_INDEX			1 
#define MS5637_PRESSURE_OFFSET_INDEX				2
#define MS5637_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX		3
#define MS5637_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX		4
#define MS5637_REFERENCE_TEMPERATURE_INDEX			5
#define MS5637_TEMP_COEFF_OF_TEMPERATURE_INDEX			6
#define MS5637_COEFFICIENT_NUMBERS				7

struct ms5637_measure_s
{
	uint32_t adc_temperature;  /* in Degree x100 */
	uint32_t adc_pressure;     /* in mBar   x10  */
	float temperature;         /* in Degree      */
	float pressure;            /* in mBar        */
};


int ms5637_register(FAR const char *devpath, FAR struct i2c_master_s *i2c, uint8_t addr);

#endif /* MS5637_H */
