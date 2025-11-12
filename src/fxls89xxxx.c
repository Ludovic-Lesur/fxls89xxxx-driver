/*
 * fxls89xxxx.c
 *
 *  Created on: 17 oct. 2025
 *      Author: Ludo
 */

#include "fxls89xxxx.h"

#ifndef FXLS89XXXX_DRIVER_DISABLE_FLAGS_FILE
#include "fxls89xxxx_driver_flags.h"
#endif
#include "error.h"
#include "maths.h"
#include "fxls89xxxx_hw.h"
#include "types.h"

#ifndef FXLS89XXXX_DRIVER_DISABLE

/*** FXLS89XXXX macros ***/

#define FXLS89XXXX_NUMBER_OF_BITS               12
#define FXLS89XXXX_WRITE_TRANSFER_SIZE_BYTES    2

/*** FXLS89XXXX local functions ***/

/*******************************************************************/
static FXLS89XXXX_status_t _FXLS89XXXX_read_register(uint8_t i2c_address, FXLS89XXXX_register_t reg_addr, uint8_t* reg_value) {
    // Local variables.
    FXLS89XXXX_status_t status = FXLS89XXXX_SUCCESS;
    uint8_t local_addr = reg_addr;
    // Check parameters.
    if (reg_addr >= FXLS89XXXX_REGISTER_LAST) {
        status = FXLS89XXXX_ERROR_REGISTER_ADDRESS;
        goto errors;
    }
    if (reg_value == NULL) {
        status = FXLS89XXXX_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Read register.
    status = FXLS89XXXX_HW_i2c_write(i2c_address, &local_addr, 1, 0);
    if (status != FXLS89XXXX_SUCCESS) goto errors;
    status = FXLS89XXXX_HW_i2c_read(i2c_address, reg_value, 1);
    if (status != FXLS89XXXX_SUCCESS) goto errors;
errors:
    return status;
}

/*** FXLS89XXXX functions ***/

/*******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_init(void) {
    // Local variables.
    FXLS89XXXX_status_t status = FXLS89XXXX_SUCCESS;
    // Init hardware interface.
    status = FXLS89XXXX_HW_init();
    if (status != FXLS89XXXX_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_de_init(void) {
    // Local variables.
    FXLS89XXXX_status_t status = FXLS89XXXX_SUCCESS;
    // Init hardware interface.
    status = FXLS89XXXX_HW_de_init();
    if (status != FXLS89XXXX_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_get_id(uint8_t i2c_address, uint8_t* chip_id) {
    // Local variables.
    FXLS89XXXX_status_t status = FXLS89XXXX_SUCCESS;
    // Read register.
    status = _FXLS89XXXX_read_register(i2c_address, FXLS89XXXX_REGISTER_WHO_AM_I, chip_id);
    if (status != FXLS89XXXX_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_write_configuration(uint8_t i2c_address, const FXLS89XXXX_register_setting_t* fxls89xxxx_configuration, uint8_t fxls89xxxx_configuration_size) {
    // Local variables.
    FXLS89XXXX_status_t status = FXLS89XXXX_SUCCESS;
    uint8_t tx_data[FXLS89XXXX_WRITE_TRANSFER_SIZE_BYTES];
    uint8_t reg_idx = 0;
    if (fxls89xxxx_configuration == NULL) {
        status = FXLS89XXXX_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (fxls89xxxx_configuration_size == 0) {
        status = FXLS89XXXX_ERROR_CONFIGURATION_SIZE;
        goto errors;
    }
    // Write configuration.
    for (reg_idx = 0; reg_idx < fxls89xxxx_configuration_size; reg_idx++) {
        // Check address.
        if (fxls89xxxx_configuration[reg_idx].address >= FXLS89XXXX_REGISTER_LAST) {
            status = FXLS89XXXX_ERROR_REGISTER_ADDRESS;
            goto errors;
        }
        // Build frame.
        tx_data[0] = (fxls89xxxx_configuration[reg_idx].address);
        tx_data[1] = (fxls89xxxx_configuration[reg_idx].value);
        // Write register.
        status = FXLS89XXXX_HW_i2c_write(i2c_address, tx_data, FXLS89XXXX_WRITE_TRANSFER_SIZE_BYTES, 1);
        if (status != FXLS89XXXX_SUCCESS) goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_clear_interrupt(uint8_t i2c_address, FXLS89XXXX_int_src1_t* int_src1, FXLS89XXXX_int_src2_t* int_src2) {
    // Local variables.
    FXLS89XXXX_status_t status = FXLS89XXXX_SUCCESS;
    // Check parameters.
    if ((int_src1 == NULL) || (int_src2 == NULL)) {
        status = FXLS89XXXX_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Read registers.
    status = _FXLS89XXXX_read_register(i2c_address, FXLS89XXXX_REGISTER_SDCD_INT_SRC1, &(int_src1->all));
    if (status != FXLS89XXXX_SUCCESS) goto errors;
    status = _FXLS89XXXX_read_register(i2c_address, FXLS89XXXX_REGISTER_SDCD_INT_SRC2, &(int_src2->all));
    if (status != FXLS89XXXX_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
FXLS89XXXX_status_t FXLS89XXXX_get_acceleration(uint8_t i2c_address, FXLS89XXXX_axis_t axis, int32_t* acceleration_data_xbits) {
    // Local variables.
    FXLS89XXXX_status_t status = FXLS89XXXX_SUCCESS;
    MATH_status_t math_status = MATH_SUCCESS;
    FXLS89XXXX_register_t reg_addr = 0;
    uint8_t reg_value = 0;
    uint32_t acceleration_data = 0;
    // Check parameters.
    if (axis >= FXLS89XXXX_AXIS_LAST) {
        status = FXLS89XXXX_ERROR_AXIS;
        goto errors;
    }
    if (acceleration_data_xbits == NULL) {
        status = FXLS89XXXX_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // MSB.
    reg_addr = (FXLS89XXXX_REGISTER_OUT_X_MSB + (axis << 1));
    status = _FXLS89XXXX_read_register(i2c_address, reg_addr, &reg_value);
    if (status != FXLS89XXXX_SUCCESS) goto errors;
    acceleration_data |= (reg_value << 8);
    // LSB.
    reg_addr--;
    status = _FXLS89XXXX_read_register(i2c_address, reg_addr, &reg_value);
    if (status != FXLS89XXXX_SUCCESS) goto errors;
    acceleration_data |= reg_value;
    // Convert to signed value.
    math_status = MATH_two_complement_to_integer(acceleration_data, (FXLS89XXXX_NUMBER_OF_BITS - 1), acceleration_data_xbits);
    MATH_exit_error(FXLS89XXXX_ERROR_BASE_MATH);
errors:
    return status;
}

#endif /* FXLS89XXXX_DRIVER_DISABLE */
