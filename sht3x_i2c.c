// allows the sht3x driver to use the pico's hardware i2c
// adapted from:
// vendor/embedded/sht/embedded-common/hw_i2c/sensirion_hw_i2c_implementatation.c
// pico-examples/i2c/ ??
/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sensirion_arch_config.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

/*
 * INSTRUCTIONS
 * ============
 *
 * Implement all functions where they are marked as IMPLEMENT.
 * Follow the function specification in the comments.
 *
 * I need to include pico-sdk
 * copy the whole sensiron into hello_usb?
 */

#define I2C_NOSTOP false
static i2c_inst_t *sensiron_i2c_selected_bus;
/**
 * Select the current i2c bus by index.
 * All following i2c operations will be directed at that bus.
 *
 * THE IMPLEMENTATION IS OPTIONAL ON SINGLE-BUS SETUPS (all sensors on the same
 * bus)
 *
 * @param bus_idx   Bus index to select
 * @returns         0 on success, an error code otherwise
 */
int16_t sensirion_i2c_select_bus(uint8_t bus_idx) {
    switch(bus_idx){
            case 0:
                    sensiron_i2c_selected_bus = i2c0;
                    return STATUS_OK;
            case 1:
                    sensiron_i2c_selected_bus = i2c1;
                    return STATUS_OK;
            default:
                    return STATUS_FAIL;
    }
}

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void gpioInit(int i) {
    gpio_set_function(i, GPIO_FUNC_I2C);
    gpio_pull_up(i);
}

void sensirion_i2c_init(void) {
    gpioInit(8);
    gpioInit(9);
    gpioInit(14);
    gpioInit(15);

    unsigned int baud = 100000; // up to 1MHz=1 000 000 for sht31
    i2c_init(i2c0, baud);
    i2c_init(i2c1, baud);
}

/**
 * Release all resources initialized by sensirion_i2c_init().
 */
void sensirion_i2c_release(void) {
    i2c_deinit(i2c0);
    i2c_deinit(i2c1);
}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count) {
    int ret = i2c_read_blocking(sensiron_i2c_selected_bus, address, data, count, I2C_NOSTOP);
    if (ret == PICO_ERROR_GENERIC) return STATUS_FAIL; else return STATUS_OK;
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_write(uint8_t address, const uint8_t* data,
                           uint16_t count) {
    int ret = i2c_write_blocking(sensiron_i2c_selected_bus, address, data, count, I2C_NOSTOP);
    if (ret == PICO_ERROR_GENERIC) return STATUS_FAIL; else return STATUS_OK;
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_sleep_usec(uint32_t useconds) {
    sleep_us(useconds);
}
