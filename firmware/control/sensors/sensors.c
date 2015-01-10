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
#include "sensors_config.h"

void sensors_init(void)
{
    int i;
    
    for (i = 0; i < SENSORS_COUNT; ++i)
        if (sensors[i].sensor_d->init != NULL && sensors[i].enabled && !(sensors[i].failed))
            sensors[i].sensor_d->init(&sensors[i]);
}

void sensors_update(void)
{
    int i;
    
    for (i = 0; i < SENSORS_COUNT; ++i)
        if (sensors[i].sensor_d->update != NULL && sensors[i].enabled && !(sensors[i].failed))
            sensors[i].sensor_d->update(&sensors[i]);
}

void sensors_sleep(void)
{
    int i;
    
    for (i = 0; i < SENSORS_COUNT; ++i)
        if (sensors[i].sensor_d->sleep != NULL && sensors[i].enabled && !(sensors[i].failed))
            sensors[i].sensor_d->sleep(&sensors[i]);
}

void sensors_wakeup(void)
{
    int i;
    
    for (i = 0; i < SENSORS_COUNT; ++i)
        if (sensors[i].sensor_d->wakeup != NULL && sensors[i].enabled && !(sensors[i].failed))
            sensors[i].sensor_d->wakeup(&sensors[i]);
}
