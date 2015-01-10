/*
 * Copyright (C) 2014 Gordon Quad
 *
 * This file is part of Glaucus Project.
 *
 * Glaucus Project is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "ch.h"
#include "hal.h"

#include "sensor.h"
#include "mpu6050.h"

bool mpu6050_init(struct sensor *sensor);
bool mpu6050_get_values(struct sensor *sensor, int32_t *vals);
bool mpu6050_reset(struct sensor *sensor);

struct sensor_desc mpu6050 = {&mpu6050_init, &mpu6050_get_values, &mpu6050_reset,
                              NULL, NULL,
                              SENSOR_GYRO_X | SENSOR_GYRO_Y | SENSOR_GYRO_Z |
                              SENSOR_ACCEL_X | SENSOR_ACCEL_Y | SENSOR_ACCEL_Z,
                              6};

bool mpu6050_init(struct sensor *sensor)
{
    I2CDriver *i2cbus = (I2CDriver *)(sensor->sensor_desc->state1);
    i2caddr_t addr = (i2caddr_t)(sensor->sensor_desc->state2);
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
    msg_t status;
    systime_t tmo = MS2ST(4);
        
    // Init MPU6050
    // CONFIG
    tx_buf[0] = 0x1A;
    tx_buf[1] = 0b00000110;
    i2cAcquireBus(i2cbus);
    status = i2cMasterTransmitTimeout(i2cbus, addr, tx_buf, 2, rx_buf, 0, tmo);
    i2cReleaseBus(i2cbus);
    if (status != RDY_OK)
        return false;
    // GYRO_CONFIG
    tx_buf[0] = 0x1B;
    tx_buf[1] = 0b00000000;
    i2cAcquireBus(i2cbus);
    status = i2cMasterTransmitTimeout(i2cbus, addr, tx_buf, 2, rx_buf, 0, tmo);
    i2cReleaseBus(i2cbus);
    if (status != RDY_OK)
        return false;
    // ACCEL_CONFIG
    tx_buf[0] = 0x1C;
    tx_buf[1] = 0b00000000;
    i2cAcquireBus(i2cbus);
    status = i2cMasterTransmitTimeout(i2cbus, addr, tx_buf, 2, rx_buf, 0, tmo);
    i2cReleaseBus(i2cbus);
    if (status != RDY_OK)
        return false;
    // PWR_MGMT_1
    tx_buf[0] = 0x6A;
    tx_buf[1] = 0b00000000;
    i2cAcquireBus(i2cbus);
    status = i2cMasterTransmitTimeout(i2cbus, addr, tx_buf, 2, rx_buf, 0, tmo);
    i2cReleaseBus(i2cbus);
    if (status != RDY_OK)
        return false;
    // PWR_MGMT_2
    tx_buf[0] = 0x6B;
    tx_buf[1] = 0b00000000;
    i2cAcquireBus(i2cbus);
    status = i2cMasterTransmitTimeout(i2cbus, addr, tx_buf, 2, rx_buf, 0, tmo);
    i2cReleaseBus(i2cbus);
    if (status != RDY_OK)
        return false;
    return true;
}

bool mpu6050_get_values(struct sensor *sensor, int32_t *vals)
{
    I2CDriver *i2cbus = (I2CDriver *)(sensor->sensor_desc->state1);
    i2caddr_t addr = (i2caddr_t)(sensor->sensor_desc->state2);
    uint8_t tx_buf[1];
    uint8_t rx_buf[12];
    msg_t status;
    systime_t tmo = MS2ST(4);

    i2cAcquireBus(i2cbus);
    tx_buf[0] = 0x3B;
    status = i2cMasterTransmitTimeout(i2cbus, addr, tx_buf, 1, rx_buf, 6, tmo);
    tx_buf[0] = 0x43;
    status = i2cMasterTransmitTimeout(i2cbus, addr, tx_buf, 1, rx_buf+6, 6, tmo);
    i2cReleaseBus(i2cbus);
    vals[0] += (int32_t)(rx_buf[7] + (rx_buf[6] << 8));
    vals[1] += (int32_t)(rx_buf[9] + (rx_buf[8] << 8));
    vals[2] += (int32_t)(rx_buf[11] + (rx_buf[10] << 8));
    vals[3] = (int32_t)(rx_buf[1] + (rx_buf[0] << 8));
    vals[4] = (int32_t)(rx_buf[3] + (rx_buf[2] << 8));
    vals[5] = (int32_t)(rx_buf[5] + (rx_buf[4] << 8));
    return true;
}

bool mpu6050_reset(struct sensor *sensor)
{
    return true;
}
