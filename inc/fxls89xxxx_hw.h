/*
 * fxls89xxxx_hw.h
 *
 *  Created on: 17 oct. 2025
 *      Author: Ludo
 */

#ifndef __FXLS89XXXX_HW_H__
#define __FXLS89XXXX_HW_H__

#ifndef FXLS89XXXX_DRIVER_DISABLE_FLAGS_FILE
#include "fxls89xxxx_driver_flags.h"
#endif
#include "fxls89xxxx.h"
#include "types.h"

#ifndef FXLS89XXXX_DRIVER_DISABLE

/*** FXLS89XXXX HW functions ***/

/*!******************************************************************
 * \fn FXLS89XXXX_status_t FXLS89XXXX_HW_init(void)
 * \brief Init FXLS89XXXX hardware interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_HW_init(void);

/*!******************************************************************
 * \fn FXLS89XXXX_status_t FXLS89XXXX_HW_de_init(void)
 * \brief Release FXLS89XXXX hardware interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_HW_de_init(void);

/*!******************************************************************
 * \fn FXLS89XXXX_status_t FXLS89XXXX_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag)
 * \brief Write data to sensor over I2C bus.
 * \param[in]   i2c_address: 7-bits sensor address.
 * \param[in]   data: Byte array to send.
 * \param[in]   data_size_bytes: Number of bytes to send.
 * \param[in]   stop_flag: Generate stop condition at the end of the transfer if non zero.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag);

/*!******************************************************************
 * \fn FXLS89XXXX_status_t FXLS89XXXX_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes)
 * \brief Read data from sensor over I2C bus.
 * \param[in]   i2c_address: 7-bits sensor address.
 * \param[in]   data_size_bytes: Number of bytes to read.
 * \param[out]  data: Byte array that will contain the read data.
 * \retval      Function execution status.
 *******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes);

#endif /* FXLS89XXXX_DRIVER_DISABLE */

#endif /* __FXLS89XXXX_HW_H__ */
