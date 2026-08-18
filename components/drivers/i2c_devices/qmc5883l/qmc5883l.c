/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * qmc5883l.c - QST QMC5883L 3-axis magnetometer driver
 *
 * Register layout differences against the HMC5883L are documented in
 * qmc5883l.h. On pyDrone this part sits on the MPU6050 AUX bus, so the
 * flight path reads it through the MPU6050 slave registers; the direct
 * accessors below only work while the MPU6050 is in I2C bypass mode.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "i2cdev.h"
#include "qmc5883l.h"
#include "stm32_legacy.h"

#define DEBUG_MODULE "QMC5883L"
#include "debug_cf.h"

static uint8_t devAddr;
static uint8_t buffer[6];
static uint8_t mode;
static I2C_Dev *I2Cx;
static bool isInit;

/** Power on and prepare for general usage.
 * Soft resets the part, then enables the internal set/reset pointer roll-over
 * that the datasheet requires before continuous mode is usable.
 */
void qmc5883lInit(I2C_Dev *i2cPort)
{
    if (isInit) {
        return;
    }

    I2Cx = i2cPort;
    devAddr = QMC5883L_ADDRESS;

    i2cdevWriteByte(I2Cx, devAddr, QMC5883L_RA_CONFIG_2, 0x00);
    vTaskDelay(M2T(100));

    /* SET/RESET period register, datasheet requires 0x01 */
    i2cdevWriteByte(I2Cx, devAddr, QMC5883L_RA_PERIOD, 0x01);
    i2cdevWriteByte(I2Cx, devAddr, QMC5883L_RA_CONFIG_2, QMC5883L_ROL_PNT);

    isInit = true;
}

void qmc5883lDeInit(void)
{
    isInit = false;
}

/** Verify the I2C connection.
 * The QMC5883L answers 0xFF on its ID register, unlike the HMC5883L's 'H43'.
 * @return True if connection is valid, false otherwise
 */
bool qmc5883lTestConnection(void)
{
    uint8_t read_id = 0;

    if (i2cdevReadByte(I2Cx, devAddr, QMC5883L_RA_CHIP_ID, &read_id)) {
        DEBUG_PRINTI("QMC5883L ID IS: 0x%X\n", read_id);
        return (read_id == QMC5883L_DEFAULT_ID);
    }

    return false;
}

/** Do a self test.
 * The QMC5883L has no bias-injection self test like the HMC5883L, so this
 * only reports whether the part was brought up.
 */
bool qmc5883lSelfTest(void)
{
    return isInit;
}

/** Set measurement mode.
 * Combined with a fixed 10Hz output rate, 2G range and 128x oversampling.
 * @param newMode QMC5883L_MODE_STANDBY or QMC5883L_MODE_CONTINUOUS
 */
void qmc5883lSetMode(uint8_t newMode)
{
    uint8_t cfg = (QMC5883L_OUTPUT_10HZ | QMC5883L_RANGE_2G | QMC5883L_SAMPLE_128);

    i2cdevWriteByte(I2Cx, devAddr, QMC5883L_RA_CONFIG_1, newMode | cfg);

    mode = newMode;
}

uint8_t qmc5883lGetMode(void)
{
    return mode;
}

/** Get 3-axis heading measurements.
 * Data registers start at 0x00 and are little endian in X, Y, Z order.
 */
void qmc5883lGetHeading(int16_t *x, int16_t *y, int16_t *z)
{
    i2cdevReadReg8(I2Cx, devAddr, QMC5883L_RA_DATAX_L, 6, buffer);

    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];
}

/** Get data ready status. */
bool qmc5883lGetReadyStatus(void)
{
    uint8_t status = 0;

    if (i2cdevReadByte(I2Cx, devAddr, QMC5883L_RA_STATUS, &status)) {
        return (status & QMC5883L_STATUS_DRDY_BIT) != 0;
    }

    return false;
}
