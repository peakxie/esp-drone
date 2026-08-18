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
 * qmc5883l.h - QST QMC5883L 3-axis magnetometer
 *
 * Despite the similar name the QMC5883L is not register compatible with the
 * Honeywell HMC5883L that the ESP-Drone reference board uses:
 *
 *                    HMC5883L          QMC5883L
 *   I2C address      0x1E              0x0D
 *   ID register      0x0A -> 'H'       0x0D -> 0xFF
 *   Data registers   0x03, big endian  0x00, little endian
 *                    X, Z, Y           X, Y, Z
 *   Data ready       0x09 bit0         0x06 bit0
 *   Sensitivity      660 LSB/Gauss     12000 LSB/Gauss (2G range)
 *
 * so the byte order and the axis order both differ - this is a separate driver
 * rather than an address tweak.
 */
#ifndef QMC5883L_H_
#define QMC5883L_H_

#include <stdbool.h>
#include <stdint.h>
#include "i2cdev.h"

#define QMC5883L_ADDRESS            0x0D
#define QMC5883L_DEFAULT_ID         0xFF

#define QMC5883L_RA_DATAX_L         0x00
#define QMC5883L_RA_DATAX_H         0x01
#define QMC5883L_RA_DATAY_L         0x02
#define QMC5883L_RA_DATAY_H         0x03
#define QMC5883L_RA_DATAZ_L         0x04
#define QMC5883L_RA_DATAZ_H         0x05
#define QMC5883L_RA_STATUS          0x06
#define QMC5883L_RA_DATA_TEMP_L     0x07
#define QMC5883L_RA_DATA_TEMP_H     0x08
#define QMC5883L_RA_CONFIG_1        0x09
#define QMC5883L_RA_CONFIG_2        0x0A
#define QMC5883L_RA_PERIOD          0x0B
#define QMC5883L_RA_CHIP_ID         0x0D

/* CONFIG_1 bits 1:0 - operating mode */
#define QMC5883L_MODE_STANDBY       0x00
#define QMC5883L_MODE_CONTINUOUS    0x01

/* CONFIG_1 bits 3:2 - output data rate */
#define QMC5883L_OUTPUT_10HZ        0x00
#define QMC5883L_OUTPUT_50HZ        0x04
#define QMC5883L_OUTPUT_100HZ       0x08
#define QMC5883L_OUTPUT_200HZ       0x0C

/* CONFIG_1 bits 5:4 - full scale range */
#define QMC5883L_RANGE_2G           0x00
#define QMC5883L_RANGE_8G           0x10

/* CONFIG_1 bits 7:6 - over sample ratio */
#define QMC5883L_SAMPLE_512         0x00
#define QMC5883L_SAMPLE_256         0x40
#define QMC5883L_SAMPLE_128         0x80
#define QMC5883L_SAMPLE_64          0xC0

/* CONFIG_2 */
#define QMC5883L_SOFT_RST           0x80
#define QMC5883L_ROL_PNT            0x40

/* STATUS */
#define QMC5883L_STATUS_DRDY_BIT    0x01
#define QMC5883L_STATUS_OVL_BIT     0x02

/* LSB per Gauss in the 2G range */
#define QMC5883L_GAUSS_PER_LSB      12000.0f

void qmc5883lInit(I2C_Dev *i2cPort);
void qmc5883lDeInit(void);
bool qmc5883lTestConnection(void);
bool qmc5883lSelfTest(void);
void qmc5883lSetMode(uint8_t mode);
uint8_t qmc5883lGetMode(void);
void qmc5883lGetHeading(int16_t *x, int16_t *y, int16_t *z);
bool qmc5883lGetReadyStatus(void);

#endif /* QMC5883L_H_ */
