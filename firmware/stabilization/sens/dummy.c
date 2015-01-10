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

#include "sensor.h"
#include "dummy.h"

bool dummy_init(struct sensor *sensor);
bool dummy_update(struct sensor *sensor);
bool dummy_reset(struct sensor *sensor);

struct sensor_desc dummy = {&dummy_init, &dummy_update, NULL, NULL, NULL};

bool dummy_init(struct sensor *sensor)
{
    return true;
}

bool dummy_get_values(struct sensor *sensor)
{
    sensor->data[0] = 0.0f;
    sensor->data[1] = 0.0f;
    sensor->data[2] = 0.0f;
    sensor->data[3] = 0.0f;
    sensor->data[4] = 0.0f;
    sensor->data[5] = 0.0f;
    sensor->data[6] = 0.0f;
    sensor->data[7] = 0.0f;
    sensor->data[8] = 0.0f;
    sensor->data[9] = 0.0f;
    return true;
}

bool dummy_reset(struct sensor *sensor)
{
    return true;
}
