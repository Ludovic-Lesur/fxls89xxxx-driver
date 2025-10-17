/*
 * fxls89xxxx_hw.c
 *
 *  Created on: 17 oct. 2025
 *      Author: Ludo
 */

#include "fxls89xxxx_hw.h"

#ifndef FXLS89XXXX_DRIVER_DISABLE_FLAGS_FILE
#include "fxls89xxxx_driver_flags.h"
#endif
#include "fxls89xxxx.h"
#include "types.h"

#ifndef FXLS89XXXX_DRIVER_DISABLE

/*** FXLS89XXXX HW functions ***/

/*******************************************************************/
FXLS89XXXX_status_t __attribute__((weak)) FXLS89XXXX_HW_init(void) {
    // Local variables.
    FXLS89XXXX_status_t status = FXLS89XXXX_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
FXLS89XXXX_status_t __attribute__((weak)) FXLS89XXXX_HW_de_init(void) {
    // Local variables.
    FXLS89XXXX_status_t status = FXLS89XXXX_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
FXLS89XXXX_status_t __attribute__((weak)) FXLS89XXXX_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    // Local variables.
    FXLS89XXXX_status_t status = FXLS89XXXX_SUCCESS;
    /* To be implemented */
    UNUSED(i2c_address);
    UNUSED(data);
    UNUSED(data_size_bytes);
    UNUSED(stop_flag);
    return status;
}

/*******************************************************************/
FXLS89XXXX_status_t __attribute__((weak)) FXLS89XXXX_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    // Local variables.
    FXLS89XXXX_status_t status = FXLS89XXXX_SUCCESS;
    /* To be implemented */
    UNUSED(i2c_address);
    UNUSED(data);
    UNUSED(data_size_bytes);
    return status;
}

#endif /* FXLS89XXXX_DRIVER_DISABLE */
