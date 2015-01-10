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

#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "ch.h"

struct sensor;

struct sensor_desc
{
    bool (*init)(struct sensor *);
    bool (*update)(struct sensor *);
    bool (*reset)(struct sensor *);
    bool (*sleep)(struct sensor *);
    bool (*wakeup)(struct sensor *);
};

// Sizes of sensor data allocated during config generations accoring to sensor 
// type
struct sensor
{
    struct sensor_desc *sensor_d;
    float *data;
    void *cal;
    void *state;
    void *init_args;
    bool enabled;
    bool failed;
};

#endif//_SENSOR_H_
