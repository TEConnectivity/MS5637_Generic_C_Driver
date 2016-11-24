/**
 * \file ms5637.h
 *
 * \brief MS5637 Temperature sensor driver header file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 *
 * \asf_license_stop
 *
 */

#ifndef MS5637_H_INCLUDED
#define MS5637_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

enum ms5637_resolution_osr {
	ms5637_resolution_osr_256 = 0,
	ms5637_resolution_osr_512,
	ms5637_resolution_osr_1024,
	ms5637_resolution_osr_2048,
	ms5637_resolution_osr_4096,
	ms5637_resolution_osr_8192
};

enum ms5637_status {
	ms5637_status_ok,
	ms5637_status_no_i2c_acknowledge,
	ms5637_status_i2c_transfer_error,
	ms5637_status_crc_error
};
	
// Functions

/**
 * \brief Configures the SERCOM I2C master to be used with the ms5637 device.
 */
void ms5637_init(void);

/**
 * \brief Check whether MS5637 device is connected
 *
 * \return bool : status of MS5637
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool ms5637_is_connected(void);

/**
 * \brief Reset the MS5637 device
 *
 * \return ms5637_status : status of MS5637
 *       - ms5637_status_ok : I2C transfer completed successfully
 *       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum ms5637_status ms5637_reset(void);

/**
 * \brief Set  ADC resolution.
 *
 * \param[in] ms5637_resolution_osr : Resolution requested
 *
 */
void ms5637_set_resolution(enum ms5637_resolution_osr );

/**
 * \brief Reads the temperature and pressure ADC value and compute the compensated values.
 *
 * \param[out] float* : Celsius Degree temperature value
 * \param[out] float* : mbar pressure value
 *
 * \return ms5637_status : status of MS5637
 *       - ms5637_status_ok : I2C transfer completed successfully
 *       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms5637_status_crc_error : CRC check error on on the PROM coefficients
 */
enum ms5637_status ms5637_read_temperature_and_pressure(float *, float *);

#endif /* MS5637_H_INCLUDED */