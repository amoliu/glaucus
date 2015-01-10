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

#ifndef _SENSORS_CONFIG_H_
#define _SENSORS_CONFIG_H_

#include "sensor.h"

#include "dummy.h"

#define SENSORS_COUNT 2
#define SENSOR_VALUES_COUNT 10

float sensor_data[SENSOR_VALUES_COUNT];

struct sensor sensors[SENSORS_COUNT] = {
    {&dummy, sensor_data + 0, NULL, NULL, NULL, true, false}
};

#endif//_SENSORS_CONFIG_H_
