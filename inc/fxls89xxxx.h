/*
 * fxls89xxxx.h
 *
 *  Created on: 17 oct. 2025
 *      Author: Ludo
 */

#ifndef __FXLS89XXXX_H__
#define __FXLS89XXXX_H__

#ifndef FXLS89XXXX_DRIVER_DISABLE_FLAGS_FILE
#include "fxls89xxxx_driver_flags.h"
#endif
#include "error.h"
#include "maths.h"
#include "types.h"

/*** FXLS89XXXX structures ***/

/*!******************************************************************
 * \enum FXLS89XXXX_status_t
 * \brief FXLS89XXXX driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    FXLS89XXXX_SUCCESS = 0,
    FXLS89XXXX_ERROR_NULL_PARAMETER,
    FXLS89XXXX_ERROR_CONFIGURATION_SIZE,
    FXLS89XXXX_ERROR_REGISTER_ADDRESS,
    FXLS89XXXX_ERROR_AXIS,
    // Low level drivers errors.
    FXLS89XXXX_ERROR_BASE_I2C = ERROR_BASE_STEP,
    FXLS89XXXX_ERROR_BASE_MATH = (FXLS89XXXX_ERROR_BASE_I2C + FXLS89XXXX_DRIVER_I2C_ERROR_BASE_LAST),
    // Last base value.
    FXLS89XXXX_ERROR_BASE_LAST = (FXLS89XXXX_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} FXLS89XXXX_status_t;

#ifndef FXLS89XXXX_DRIVER_DISABLE

/*!******************************************************************
 * \enum FXLS89XXXX_register_t
 * \brief FXLS89XXXX registers set.
 *******************************************************************/
typedef enum {
    FXLS89XXXX_REGISTER_INT_STATUS = 0x00,
    FXLS89XXXX_REGISTER_TEMP_OUT = 0x01,
    FXLS89XXXX_REGISTER_VECM_LSB = 0x02,
    FXLS89XXXX_REGISTER_VECM_MSB = 0x03,
    FXLS89XXXX_REGISTER_OUT_X_LSB = 0x04,
    FXLS89XXXX_REGISTER_OUT_X_MSB = 0x05,
    FXLS89XXXX_REGISTER_OUT_Y_LSB = 0x06,
    FXLS89XXXX_REGISTER_OUT_Y_MSB = 0x07,
    FXLS89XXXX_REGISTER_OUT_Z_LSB = 0x08,
    FXLS89XXXX_REGISTER_OUT_Z_MSB = 0x09,
    FXLS89XXXX_REGISTER_BUF_STATUS = 0x0B,
    FXLS89XXXX_REGISTER_BUF_X_LSB = 0x0C,
    FXLS89XXXX_REGISTER_BUF_X_MSB = 0x0D,
    FXLS89XXXX_REGISTER_BUF_Y_LSB = 0x0E,
    FXLS89XXXX_REGISTER_BUF_Y_MSB = 0x0F,
    FXLS89XXXX_REGISTER_BUF_Z_LSB = 0x10,
    FXLS89XXXX_REGISTER_BUF_Z_MSB = 0x11,
    FXLS89XXXX_REGISTER_PROD_REV = 0x12,
    FXLS89XXXX_REGISTER_WHO_AM_I = 0x13,
    FXLS89XXXX_REGISTER_SYS_MODE = 0x14,
    FXLS89XXXX_REGISTER_SENS_CONFIG1 = 0x15,
    FXLS89XXXX_REGISTER_SENS_CONFIG2 = 0x16,
    FXLS89XXXX_REGISTER_SENS_CONFIG3 = 0x17,
    FXLS89XXXX_REGISTER_SENS_CONFIG4 = 0x18,
    FXLS89XXXX_REGISTER_SENS_CONFIG5 = 0x19,
    FXLS89XXXX_REGISTER_WAKE_IDLE_LSB = 0x1A,
    FXLS89XXXX_REGISTER_WAKE_IDLE_MSB = 0x1B,
    FXLS89XXXX_REGISTER_SLEEP_IDLE_LSB = 0x1C,
    FXLS89XXXX_REGISTER_SLEEP_IDLE_MSB = 0x1D,
    FXLS89XXXX_REGISTER_ASLP_COUNT_LSB = 0x1E,
    FXLS89XXXX_REGISTER_ASLP_COUNT_MSB = 0x1F,
    FXLS89XXXX_REGISTER_INT_EN = 0x20,
    FXLS89XXXX_REGISTER_INT_PIN_SEL = 0x21,
    FXLS89XXXX_REGISTER_OFF_X = 0x22,
    FXLS89XXXX_REGISTER_OFF_Y = 0x23,
    FXLS89XXXX_REGISTER_OFF_Z = 0x24,
    FXLS89XXXX_REGISTER_BUF_CONFIG1 = 0x26,
    FXLS89XXXX_REGISTER_BUF_CONFIG2 = 0x27,
    FXLS89XXXX_REGISTER_ORIENT_STATUS = 0x28,
    FXLS89XXXX_REGISTER_ORIENT_CONFIG = 0x29,
    FXLS89XXXX_REGISTER_ORIENT_DBCOUNT = 0x2A,
    FXLS89XXXX_REGISTER_ORIENT_BF_ZCOMP = 0x2B,
    FXLS89XXXX_REGISTER_ORIENT_THS_REG = 0x2C,
    FXLS89XXXX_REGISTER_SDCD_INT_SRC1 = 0x2D,
    FXLS89XXXX_REGISTER_SDCD_INT_SRC2 = 0x2E,
    FXLS89XXXX_REGISTER_SDCD_CONFIG1 = 0x2F,
    FXLS89XXXX_REGISTER_SDCD_CONFIG2 = 0x30,
    FXLS89XXXX_REGISTER_SDCD_OT_DBCOUNT = 0x31,
    FXLS89XXXX_REGISTER_SDCD_WT_DBCOUNT = 0x32,
    FXLS89XXXX_REGISTER_SDCD_LTHS_LSB = 0x33,
    FXLS89XXXX_REGISTER_SDCD_LTHS_MSB = 0x34,
    FXLS89XXXX_REGISTER_SDCD_UTHS_LSB = 0x35,
    FXLS89XXXX_REGISTER_SDCD_UTHS_MSB = 0x36,
    FXLS89XXXX_REGISTER_SELF_TEST_CONFIG1 = 0x37,
    FXLS89XXXX_REGISTER_SELF_TEST_CONFIG2 = 0x38,
    FXLS89XXXX_REGISTER_LAST
} FXLS89XXXX_register_t;

/*!******************************************************************
 * \struct FXLS89XXXX_register_setting_t
 * \brief FXLS89XXXX register structure.
 *******************************************************************/
typedef struct {
    uint8_t address;
    uint8_t value;
} FXLS89XXXX_register_setting_t;

/*!******************************************************************
 * \struct FXLS89XXXX_int_src1_t
 * \brief FXLS89XXXX INT_SRC1 register format.
 *******************************************************************/
typedef union {
    uint8_t all;
    struct {
        unsigned ot_ea :1;
        unsigned reserved6 :1;
        unsigned x_ot_ef :1;
        unsigned x_ot_pol :1;
        unsigned y_ot_ef :1;
        unsigned y_ot_pol :1;
        unsigned z_ot_ef :1;
        unsigned z_ot_pol :1;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} FXLS89XXXX_int_src1_t;

/*!******************************************************************
 * \struct FXLS89XXXX_int_src2_t
 * \brief FXLS89XXXX INT_SRC2 register format.
 *******************************************************************/
typedef union {
    uint8_t all;
    struct {
        unsigned wt_ea :1;
        unsigned reserved6 :1;
        unsigned x_wt_ef :1;
        unsigned reserved4 :1;
        unsigned y_wt_ef :1;
        unsigned reserved2 :1;
        unsigned z_wt_ef :1;
        unsigned reserved0 :1;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} FXLS89XXXX_int_src2_t;

/*!******************************************************************
 * \enum FXLS89XXXX_axis_t
 * \brief FXLS89XXXX axis list.
 *******************************************************************/
typedef enum {
    FXLS89XXXX_AXIS_X = 0,
    FXLS89XXXX_AXIS_Y,
    FXLS89XXXX_AXIS_Z,
    FXLS89XXXX_AXIS_LAST
} FXLS89XXXX_axis_t;

/*** FXLS89XXXX functions ***/

/*!******************************************************************
 * \fn FXLS89XXXX_status_t FXLS89XXXX_init(void)
 * \brief Init FXLS89XXXX interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_init(void);

/*!******************************************************************
 * \fn FXLS89XXXX_status_t FXLS89XXXX_de_init(void)
 * \brief Release FXLS89XXXX interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_de_init(void);

/*!******************************************************************
 * \fn FXLS89XXXX_status_t FXLS89XXXX_get_id(uint8_t i2c_address, uint8_t* chip_id)
 * \brief Read accelerometer chip ID.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[in]   none
 * \param[out]  chip_id: Pointer to the read chip ID.
 * \retval      Function execution status.
 *******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_get_id(uint8_t i2c_address, uint8_t* chip_id);

/*!******************************************************************
 * \fn FXLS89XXXX_status_t FXLS89XXXX_write_configuration(uint8_t i2c_address, const FXLS89XXXX_register_setting_t* fxls89xxxx_configuration, uint8_t fxls89xxxx_configuration_size)
 * \brief Set accelerometer configuration.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[in]   fxls89xxxx_config: List of registers and values to set.
 * \param[in]   fxls89xxxx_config_size: Size of the configuration.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_write_configuration(uint8_t i2c_address, const FXLS89XXXX_register_setting_t* fxls89xxxx_configuration, uint8_t fxls89xxxx_configuration_size);

/*!******************************************************************
 * \fn FXLS89XXXX_status_t FXLS89XXXX_clear_interrupt(uint8_t i2c_address, FXLS89XXXX_int_src1_t* int_src1, FXLS89XXXX_int_src2_t* int_src2)
 * \brief Clear accelerometer interrupt and read corresponding source.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[out]  int_src1: Pointer to byte that will contain the INT_SRC1 register value.
 * \param[out]  int_src2: Pointer to byte that will contain the INT_SRC2 register value.
 * \retval      Function execution status.
 *******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_clear_interrupt(uint8_t i2c_address, FXLS89XXXX_int_src1_t* int_src1, FXLS89XXXX_int_src2_t* int_src2);

/*!******************************************************************
 * \fn FXLS89XXXX_status_t FXLS89XXXX_get_acceleration(uint8_t i2c_address, FXLS89XXXX_axis_t axis, int32_t* acceleration_data_xbits)
 * \brief Read raw accelerometer data.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[in]   axis: Accelerometer axis to read.
 * \param[out]  acceleration_data_xbits: Pointer that will contain the axis acceleration value.
 * \retval      Function execution status.
 *******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_get_acceleration(uint8_t i2c_address, FXLS89XXXX_axis_t axis, int32_t* acceleration_data_xbits);

/*******************************************************************/
#define FXLS89XXXX_exit_error(base) { ERROR_check_exit(fxls89xxxx_status, FXLS89XXXX_SUCCESS, base) }

/*******************************************************************/
#define FXLS89XXXX_stack_error(base) { ERROR_check_stack(fxls89xxxx_status, FXLS89XXXX_SUCCESS, base) }

/*******************************************************************/
#define FXLS89XXXX_stack_exit_error(base, code) { ERROR_check_stack_exit(fxls89xxxx_status, FXLS89XXXX_SUCCESS, base, code) }

#endif /* FXLS89XXXX_DRIVER_DISABLE */

#endif /* __FXLS89XXXX_H__ */
