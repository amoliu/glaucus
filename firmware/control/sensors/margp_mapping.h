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

#ifndef _MARGP_MAPPING_H_
#define _MARGP_MAPPING_H_

#include "sensors_config.h"

#include "margp.h"

struct margp_mapping margp_mapping = {
    // GYRO_X
    {sensor_data + 0, NULL, NULL}, true,
    // GYRO_Y
    {sensor_data + 1, NULL, NULL}, true,
    // GYRO_Z
    {sensor_data + 2, NULL, NULL}, true,
    // ACCEL_X
    {sensor_data + 3, NULL, NULL}, true,
    // ACCEL_Y
    {sensor_data + 4, NULL, NULL}, true,
    // ACCEL_Z
    {sensor_data + 5, NULL, NULL}, true,
    // MAG_X
    {sensor_data + 6, NULL, NULL}, true,
    // MAG_Y
    {sensor_data + 7, NULL, NULL}, true,
    // MAG_Z
    {sensor_data + 8, NULL, NULL}, true,
    // PRES
    {sensor_data + 9, NULL, NULL}, true,
};

#endif//_MARGP_MAPPING_H_
