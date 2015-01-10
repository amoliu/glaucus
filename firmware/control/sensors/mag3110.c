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
#include "mag3110.h"

bool mag3110_init(struct sensor *sensor);
bool mag3110_get_values(struct sensor *sensor, int32_t *vals);
bool mag3110_reset(struct sensor *sensor);

struct sensor_desc mag3110 = {&mag3110_init, &mag3110_get_values, &mag3110_reset,
                              NULL, NULL,
                              SENSOR_MAG_X | SENSOR_MAG_Y | SENSOR_MAG_Z,
                              3};

bool mag3110_init(struct sensor *sensor)
{
    I2CDriver *i2cbus = (I2CDriver *)(sensor->sensor_desc->state1);
    i2caddr_t addr = (i2caddr_t)(sensor->sensor_desc->state2);
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
    msg_t status;
    systime_t tmo = MS2ST(4);
        
    // Init MAG3110
    tx_buf[0] = 0x11;
    tx_buf[1] = 0x80;
    i2cAcquireBus(i2cbus);
    status = i2cMasterTransmitTimeout(i2cbus, addr, tx_buf, 2, rx_buf, 0, tmo);
    i2cReleaseBus(i2cbus);
    if (status != RDY_OK)
        return false;
    tx_buf[0] = 0x10;
    tx_buf[1] = 0x01;
    i2cAcquireBus(i2cbus);
    status = i2cMasterTransmitTimeout(i2cbus, addr, tx_buf, 2, rx_buf, 0, tmo);
    i2cReleaseBus(i2cbus);
    if (status != RDY_OK)
        return false;
    return true;
}

bool mag3110_get_values(struct sensor *sensor, int32_t *vals)
{
    I2CDriver *i2cbus = (I2CDriver *)(sensor->sensor_desc->state1);
    i2caddr_t addr = (i2caddr_t)(sensor->sensor_desc->state2);
    uint8_t tx_buf[1];
    uint8_t rx_buf[12];
    msg_t status;
    systime_t tmo = MS2ST(4);

    i2cAcquireBus(i2cbus);
    tx_buf[0] = 0x01;
    status = i2cMasterTransmitTimeout(i2cbus, addr, tx_buf, 1, rx_buf, 6, tmo);
    i2cReleaseBus(i2cbus);
    vals[0] = (int32_t)(rx_buf[1] + (rx_buf[0] << 8));
    vals[1] = (int32_t)(rx_buf[3] + (rx_buf[2] << 8));
    vals[2] = (int32_t)(rx_buf[5] + (rx_buf[4] << 8));
    return true;
}

bool mag3110_reset(struct sensor *sensor)
{
    return true;
}
