/**
 *
 * MS5637 Temperature sensor driver source file
 *
 * Copyright (c) 2020 James F Dougherty <jafrado@gmail.com>
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * For details on programming, refer to ms5637 datasheet :
 * http://www.meas-spec.com/downloads/MS5637-02BA03.pdf
 *
 */
#include <nuttx/config.h>
#include <errno.h>
#include <debug.h>
#include <stdlib.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ms5637.h>
#include <nuttx/random.h>
#include <memory.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MS5637)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MS5637_I2C_FREQUENCY
#define CONFIG_MS5637_I2C_FREQUENCY 100000
#endif


/* MS5637 dev */
struct ms5637_dev_s {
	FAR struct ms5637_dev_s *flink; /* Linked list of drivers */
	FAR struct i2c_master_s *i2c;   /* I2C interface */
	uint8_t                 addr;   /* I2C address */
	int                     freq;   /* MS5637 I2C Frequency    */
	uint32_t     adc_temperature;   /* Uncompensated temp (Centigrade) */
	uint32_t        adc_pressure;   /* Uncompensated pressure (millibar) */
	float            temperature;   /* in Degree      */
	float               pressure;   /* in mBar        */
	uint8_t                  osr;   /* Oversampling ratio bits */
	useconds_t             delay;   /* Oversampling ratio delay */
	sem_t                datasem;   /* Manages exclusive access */
	int               coeff_read;   /* EEPROM read or not */
};

// Static functions
static int ms5637_write_command(FAR struct ms5637_dev_s*, uint8_t);
static int ms5637_read_eeprom_coeff(FAR struct ms5637_dev_s*, uint8_t, uint16_t*);
static int ms5637_read_eeprom(FAR struct ms5637_dev_s*);
static int ms5637_conversion_and_read_adc(FAR struct ms5637_dev_s* dev, uint8_t cmd, uint32_t *adc);
static int ms5637_crc_check (FAR struct ms5637_dev_s* dev, uint16_t *n_prom, uint8_t crc);


static uint16_t eeprom_coeff[MS5637_COEFFICIENT_NUMBERS];
static uint32_t conversion_time[6] = {	MS5637_CONVERSION_TIME_OSR_256,
					MS5637_CONVERSION_TIME_OSR_512,
					MS5637_CONVERSION_TIME_OSR_1024,
					MS5637_CONVERSION_TIME_OSR_2048,
					MS5637_CONVERSION_TIME_OSR_4096,
					MS5637_CONVERSION_TIME_OSR_8192};

/* Character Driver Methods */
static int     ms5637_open(FAR struct file *filep);
static int     ms5637_close(FAR struct file *filep);
static ssize_t ms5637_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t ms5637_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     ms5637_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fops =
{
	ms5637_open,
	ms5637_close,
	ms5637_read,
	ms5637_write,
	NULL,
	ms5637_ioctl
#ifndef CONFIG_DISABLE_POLL
	, NULL
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
	, NULL
#endif
};

/**
 * \brief Reset the MS5637 device
 *
 * \return ms5637_status : status of MS5637
 *       - MS5637_OK : I2C transfer completed successfully
 *       - MS5637_ENXIO : Problem with i2c transfer
 *       - MS5637_NO_I2C_ACK : I2C did not acknowledge
 */
static int ms5637_reset(FAR struct ms5637_dev_s* dev)
{
	int ret = 0;
	ret = ms5637_write_command(dev, MS5637_RESET_COMMAND);
	up_mdelay(500);
	return ret;
}

/**
 * \brief Writes the MS5637 8-bits command with the value passed
 *
 * \param[in] uint8_t : Command value to be written.
 *
 * \return ms5637_status : status of MS5637
 *       - MS5637_OK : I2C transfer completed successfully
 *       - MS5637_ENXIO : Problem with i2c transfer
 *       - MS5637_NO_I2C_ACK : I2C did not acknowledge
 */
static int ms5637_write_command(FAR struct ms5637_dev_s* dev,uint8_t cmd)
{
	struct i2c_msg_s msg;
	int ret;
	
	/* Setup for the transfer */	
	msg.frequency = CONFIG_MS5637_I2C_FREQUENCY;
	msg.addr      = dev->addr;
	msg.flags     = I2C_M_NORESTART;	
	msg.buffer    = (FAR uint8_t *)&cmd;  /* Override const */
	msg.length    = 1;
	/* Then perform the transfer. */
	ret = I2C_TRANSFER(dev->i2c, &msg, 1);
	return (ret >= 0) ? OK : ret;
}

/**
 * \brief Reads the ms5637 EEPROM coefficient stored at address provided.
 *
 * \param[in] uint8_t : Address of coefficient in EEPROM
 * \param[out] uint16_t* : Value read in EEPROM
 *
 * \return ms5637_status : status of MS5637
 *       - MS5637_OK : I2C transfer completed successfully
 *       - MS5637_ENXIO : Problem with i2c transfer
 *       - MS5637_NO_I2C_ACK : I2C did not acknowledge
 *       - MS5637_CRC_ERROR : CRC check error on the coefficients
 */
static int ms5637_read_eeprom_coeff(FAR struct ms5637_dev_s* dev, uint8_t command, uint16_t *coeff)
{
	struct i2c_msg_s msg;
	int status;
	uint8_t buffer[2];
	
	buffer[0] = 0;
	buffer[1] = 0;

	/* Send the conversion command */
	status = ms5637_write_command(dev, command);
	if(status != OK)
		return status;
	/* Setup for the transfer */	
	msg.frequency = CONFIG_MS5637_I2C_FREQUENCY;
	msg.addr      = dev->addr;
	msg.flags     = I2C_M_READ;
	msg.buffer    = (FAR uint8_t *)&buffer[0];  /* Override const */
	msg.length    = 2;
	/* Perform the transfer to read coefficient */
	status = I2C_TRANSFER(dev->i2c, &msg, 1);
	*coeff = (buffer[0] << 8) | buffer[1];
	sninfo("coeff: 0x%02x\n", *coeff);
	/* guard against bogus data */
	if (*coeff == 0)
		status = MS5637_ENXIO;
	return (status >= 0) ? OK : status;
}

/**
 * \brief Reads the ms5637 EEPROM coefficients to store them for computation.
 *
 * \return ms5637_status : status of MS5637
 *       - MS5637_OK : I2C transfer completed successfully
 *       - MS5637_ENXIO : Problem with i2c transfer
 *       - MS5637_NO_I2C_ACK : I2C did not acknowledge
 *       - MS5637_CRC_ERROR : CRC check error on the coefficients
 */
static int ms5637_read_eeprom(FAR struct ms5637_dev_s* dev)
{
	int status;
	uint8_t i;
	
	for( i=0 ; i< MS5637_COEFFICIENT_NUMBERS ; i++)
	{
		status = ms5637_read_eeprom_coeff(dev, MS5637_PROM_ADDRESS_READ_ADDRESS_0 + i * 2, eeprom_coeff + i);
		if(status != OK)
			return status;
	}
    
	if( !ms5637_crc_check(dev, eeprom_coeff, ( eeprom_coeff[MS5637_CRC_INDEX] & 0xF000 ) >> 12) )
		return MS5637_CRC_ERROR;
	
	dev->coeff_read = 1;
	
	return OK;
}

/**
 * \brief Triggers conversion and read ADC value
 *
 * \param[in] uint8_t : Command used for conversion (will determine Temperature vs Pressure and osr)
 * \param[out] uint32_t* : ADC value.
 *
 * \return ms5637_status : status of MS5637
 *       - MS5637_OK : I2C transfer completed successfully
 *       - MS5637_ENXIO : Problem with i2c transfer
 *       - MS5637_NO_I2C_ACK : I2C did not acknowledge
 */
static int ms5637_conversion_and_read_adc(FAR struct ms5637_dev_s* dev, uint8_t cmd, uint32_t *adc)
{
	struct i2c_msg_s msg;
	int status;
	uint8_t buffer[3], c2;
	
	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	//send conversion command
	msg.frequency = CONFIG_MS5637_I2C_FREQUENCY;
	msg.addr      = dev->addr;
	msg.flags     = 0;	
	msg.buffer    = (FAR uint8_t *)&cmd;  /* Override const */
	msg.length    = 1;
	status = I2C_TRANSFER(dev->i2c, &msg, 1); /* Perform the transfer. */
	if (status < 0) {
		sninfo("bad command status\n");		
		return status;
	}
	sninfo("osr delay:%d ms\n", conversion_time[ (cmd & MS5637_CONVERSION_OSR_MASK)/2 ]/1000 );
	
        // delay conversion depending on resolution
	up_mdelay( 10 * conversion_time[ (cmd & MS5637_CONVERSION_OSR_MASK)/2 ]/1000 );

	//start 24-bit read after delay time
	c2 = MS5637_READ_ADC;
	msg.frequency = CONFIG_MS5637_I2C_FREQUENCY;
	msg.addr      = dev->addr;
	msg.flags     = I2C_M_NORESTART;	
	msg.buffer    = (FAR uint8_t *)&c2;       /* Override const */
	msg.length    = 1;
	status = I2C_TRANSFER(dev->i2c, &msg, 1); /* Perform the transfer. */
	if (status < 0) {
		sninfo("bad adc read status: %d\n", status);
		return status;
	}	
	/* Read 24-bit result */
	msg.frequency = CONFIG_MS5637_I2C_FREQUENCY;
	msg.addr      = dev->addr;
	msg.flags     = I2C_M_READ;
	msg.buffer    = (FAR uint8_t *)&buffer[0];  /* Override const */	
	msg.length    = 3;	

	/* finally perform the transfer. */
	status = I2C_TRANSFER(dev->i2c, &msg, 1);
	if (status < 0) {
		sninfo("bad adc 24-bit read : %d\n", status);
	}
	*adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
	
	return (status >= 0) ? OK : status;
}

/**
 * \brief Reads the temperature and pressure ADC value and compute the compensated values.
 *
 * \param[out] float* : Celsius Degree temperature value
 * \param[out] float* : mbar pressure value
 *
 * \return ms5637_status : status of MS5637
 *       - MS5637_OK : I2C transfer completed successfully
 *       - MS5637_ENXIO : Problem with i2c transfer
 *       - MS5637_NO_I2C_ACK : I2C did not acknowledge
 *       - MS5637_CRC_ERROR : CRC check error on the coefficients
 */
static int ms5637_read_temperature_and_pressure(FAR struct ms5637_dev_s* dev)
{
	int status = MS5637_OK;
	uint32_t adc_temperature, adc_pressure;
	int32_t dT, TEMP;
	int64_t OFF, SENS, P, T2, OFF2, SENS2;
	uint8_t cmd;
	
	// If first time adc is requested, get EEPROM coefficients
	if( !dev->coeff_read ) 
		status = ms5637_read_eeprom(dev);
	if( status != OK) {
		sninfo("MS5637: could not read eeprom\n");
		return status;
	}
	
	// First read temperature
	cmd = dev->osr * 2;
	cmd |= MS5637_START_TEMPERATURE_ADC_CONVERSION;
	status = ms5637_conversion_and_read_adc(dev, cmd, &adc_temperature);
	if( status != OK) {
		sninfo("MS5637: could not read temperature\n");
		return status;
	}

	// Now read pressure
	cmd = dev->osr * 2;
	cmd |= MS5637_START_PRESSURE_ADC_CONVERSION;
	status = ms5637_conversion_and_read_adc(dev, cmd, &adc_pressure);
	if( status != OK) {
		sninfo("MS5637: could not read Pressure\n");		
		return status;
	}
    
	if ((adc_temperature == 0) || (adc_pressure == 0)){
		sninfo("MS5637: zero values\n");
		return MS5637_ENXIO;
	}
	dev->adc_pressure = adc_pressure;
	dev->adc_temperature = adc_temperature;
	// Difference between actual and reference temperature = D2 - Tref
	dT = (int32_t)adc_temperature -((int32_t)eeprom_coeff[MS5637_REFERENCE_TEMPERATURE_INDEX] <<8 );
	
	// Actual temperature = 2000 + dT * TEMPSENS
	TEMP = 2000+((int64_t)dT *(int64_t)eeprom_coeff[MS5637_TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23) ;
	
	// Second order temperature compensation
	if( TEMP < 2000 )
	{
		T2 = ( 3 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 33;
		OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		
		if( TEMP < -1500 )
		{
			OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
			SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
		}
	}
	else
	{
		T2 = ( 5 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 38;
		OFF2 = 0 ;
		SENS2 = 0 ;
	}
	
	// OFF = OFF_T1 + TCO * dT
	OFF = ( (int64_t)(eeprom_coeff[MS5637_PRESSURE_OFFSET_INDEX]) << 17 ) + ( ( (int64_t)(eeprom_coeff[MS5637_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) * dT ) >> 6 ) ;
	OFF -= OFF2 ;
	
	// Sensitivity at actual temperature = SENS_T1 + TCS * dT
	SENS = ( (int64_t)eeprom_coeff[MS5637_PRESSURE_SENSITIVITY_INDEX] << 16 ) + ( ((int64_t)eeprom_coeff[MS5637_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dT) >> 7 ) ;
	SENS -= SENS2 ;
	
	// Temperature compensated pressure = D1 * SENS - OFF
	P = ( ( (adc_pressure * SENS) >> 21 ) - OFF ) >> 15 ;
	
	dev->temperature = ( (float)TEMP - T2 ) / 100;
	dev->pressure = (float)P / 100;
	sninfo("MS5637: ADC: %d, Temp %d\n", adc_pressure, adc_temperature);
	
	add_sensor_randomness(adc_pressure ^ adc_temperature);
	
	return status;
}

/**
 * \brief CRC check
 *
 * \param[in] uint16_t *: List of EEPROM coefficients
 * \param[in] uint8_t : crc to compare with
 *
 * \return int : TRUE if CRC is OK, FALSE if KO
 */
static int ms5637_crc_check (FAR struct ms5637_dev_s* dev, uint16_t *n_prom, uint8_t crc)
{
	uint8_t cnt, n_bit; 
	uint16_t n_rem; 
	uint16_t crc_read;
	
	n_rem = 0x00;
	crc_read = n_prom[7]; 
	n_prom[7] = (0xFF00 & (n_prom[7])); 
	for (cnt = 0; cnt < 16; cnt++) 
	{
		if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
		else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
		for (n_bit = 8; n_bit > 0; n_bit--)
		{
			if (n_rem & (0x8000))
				n_rem = (n_rem << 1) ^ 0x3000;
			else
				n_rem = (n_rem << 1);
		}
	}
	n_rem = (0x000F & (n_rem >> 12)); 
	n_prom[7] = crc_read;
	n_rem ^= 0x00;
        
	return  ( n_rem == crc );
}

/****************************************************************************
 * Name: ms5637_open
 *
 * Description:
 *   This method is called when the device is opened.
 *
 ****************************************************************************/
static int ms5637_open(FAR struct file *filep)
{
	return OK;
}

/****************************************************************************
 * Name: ms5637_close
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/
static int ms5637_close(FAR struct file *filep)
{
	return OK;
}

/****************************************************************************
 * Name: ms5637_read
 *
 * Description:
 *   A dummy read method.
 *
 ****************************************************************************/
static ssize_t ms5637_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
	FAR struct inode *inode        = filep->f_inode;
	FAR struct ms5637_dev_s *priv  = inode->i_private;
	FAR struct ms5637_measure_s *p = (FAR struct ms5637_measure_s *)buffer;

	/* Copy the sensor data into the buffer */
	memset(p, 0, sizeof(FAR struct ms5637_measure_s));
	
	if (ms5637_read_temperature_and_pressure(priv) < 0) {
		return 0;
	}
		
	p->temperature  = priv->temperature;
	p->pressure     = priv->pressure;
	p->adc_temperature  = priv->adc_temperature;
	p->adc_pressure     = priv->adc_pressure;
	
	return sizeof(FAR struct ms5637_measure_s);
}

/****************************************************************************
 * Name: ms5637_write
 *
 * Description:
 *   A dummy write method.
 *
 ****************************************************************************/
static ssize_t ms5637_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
	return -ENOSYS;
}

/****************************************************************************
 * Name: ms5637_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/
static int ms5637_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
	FAR struct inode        *inode = filep->f_inode;
	FAR struct ms5637_dev_s *priv  = inode->i_private;
	int                      ret   = OK;
	
	/* Handle ioctl commands */
	
	switch (cmd)
	{
		/* Measure the temperature and the pressure. Arg: None. */
		
	case SNIOC_MEASURE:
		DEBUGASSERT(arg == 0);
		ret = ms5637_read_temperature_and_pressure(priv);
		break;
		
		/* Return the temperature last measured. Arg: int32_t* pointer. */
		
	case SNIOC_TEMPERATURE:
        {
		FAR int32_t *ptr = (FAR int32_t *)((uintptr_t)arg);
		DEBUGASSERT(ptr != NULL);
		*ptr = priv->adc_temperature;
		sninfo("temp: %08x\n", *ptr);
        }
        break;
	
	/* Return the pressure last measured. Arg: int32_t* pointer. */
	
	case SNIOC_PRESSURE:
        {
		FAR int32_t *ptr = (FAR int32_t *)((uintptr_t)arg);
		DEBUGASSERT(ptr != NULL);
		*ptr = priv->adc_pressure;
		sninfo("press: %08x\n", *ptr);
        }
        break;
	
	/* Reset the device. Arg: None. */
	
	case SNIOC_RESET:
		DEBUGASSERT(arg == 0);
		ret = ms5637_reset(priv);
		break;
		
		/* Change the oversampling ratio. Arg: uint16_t value. */
		
	case SNIOC_OVERSAMPLING:
		priv->osr = (uint16_t)arg;
		sninfo("osr: %04x\n", *(uint16_t *)arg);
		break;
		
		/* Unrecognized commands */		
	default:
		snerr("ERROR: Unrecognized cmd: %d arg: %ld\n", cmd, arg);
		ret = -ENOTTY;
		break;
	}
	
	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ms5637_register
 *
 * Description:
 *   Register the MS5637 character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/press0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the MS5637.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int ms5637_register(FAR const char *devpath, FAR struct i2c_master_s *i2c, uint8_t addr)
{
	FAR struct ms5637_dev_s *priv;
	int ret;

	/* Sanity check */
	DEBUGASSERT(i2c != NULL);

	/* Initialize the device's structure */
	priv = (FAR struct ms5637_dev_s *)kmm_malloc(sizeof(*priv));
	if (priv == NULL)
	{
		snerr("ERROR: Failed to allocate instance\n");
		return -ENOMEM;
	}

	priv->i2c   = i2c;
	priv->addr  = addr;
	priv->temperature  = 0;
	priv->pressure = 0;
	priv->adc_temperature = 0;
	priv->adc_pressure = 0;
	priv->osr = MS5637_RESOLUTION_OSR_8192;
//	priv->osr = MS5637_RESOLUTION_OSR_256;
	priv->coeff_read = 0;	
	
	ret = ms5637_reset(priv);
	if (ret < 0)
	{
		snerr("ERROR: ms5637_reset failed: %d\n", ret);
		goto errout;
	}
	
	/* Register the character driver */
	ret = register_driver(devpath, &g_fops, 0666, priv);
	if (ret < 0)
	{
		snerr("ERROR: Failed to register driver: %d\n", ret);
		goto errout;
	}
	
	return ret;
	
errout:
	kmm_free(priv);
	return ret;
}

#endif /* ! defined(CONFIG_I2C) && !defined(CONFIG_SENSORS_MS5637) */
