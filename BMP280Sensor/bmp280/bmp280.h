#ifndef BMP280_H_
#define BMP280_H_
/*
 * Library for Bosch BMP280 pressure sensor for AVR MCUs in plain old C.
 *
 * This library uses the I2C layer by Peter Fleury, and is loosely modelled
 * after the bmp085 library by Davide Gironi.
 * 
 * The library is distributable under the terms of the GNU General
 * Public License, version 2 only.
 *
 * Written by Jan "Yenya" Kasprzak, https://www.fi.muni.cz/~kas/
 *
 * Configuration can be done by editing the top of the bmp280.c file.
 */

// Oversampling rate for the sensor.
#define SAMPLING_NONE 0x00
#define SAMPLING_X1 0x01
#define SAMPLING_X2 0x02
#define SAMPLING_X4 0x03
#define SAMPLING_X8 0x04
#define SAMPLING_X16 0x05

// Operating mode for the sensor.
#define MODE_SLEEP 0x00
#define MODE_FORCED 0x01
#define MODE_NORMAL 0x03

// Filtering level for sensor data.
#define FILTER_OFF 0x00
#define FILTER_X2 0x01
#define FILTER_X4 0x02
#define FILTER_X8 0x03
#define FILTER_X16 0x04

// Standby duration in ms
#define STANDBY_MS_1 0x00
#define STANDBY_MS_63 0x01
#define STANDBY_MS_125 0x02
#define STANDBY_MS_250 0x03
#define STANDBY_MS_500 0x04
#define STANDBY_MS_1000 0x05
#define STANDBY_MS_2000 0x06
#define STANDBY_MS_4000 0x07


uint8_t bmp280_init(void);		// call this first
uint8_t bmp280_get_status(void);	// read the status register
void bmp280_set_config(uint8_t t_sb, uint8_t filter, uint8_t spi3w_en);
	// set the configuration register
void bmp280_set_ctrl(uint8_t osrs_t, uint8_t osrs_p, uint8_t mode);
	// set the oversampling and mode (this starts the conversion)

void bmp280_setSampling(uint8_t mode, uint8_t tempSampling, uint8_t pressSampling, uint8_t filter, uint8_t duration);

void bmp280_takeForcedMeasurement(uint8_t mode, uint8_t tempSampling, uint8_t pressSampling);

void bmp280_measure(void);		// do a measurement

// the following functions return the result of the last measurement
#define bmp280_getpressure()	(_bmp280_pres)
#define bmp280_gettemperature()	(_bmp280_temp)
double bmp280_getaltitude(void);

// do not use directly, call the macros above
extern int32_t _bmp280_temp;
extern uint32_t _bmp280_pres;

#endif /* BMP280_H_ */
