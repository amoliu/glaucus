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

#include <math.h>

#include "margp.h"
#include "margp_mapping.h"

struct margp_values margp;

void margp_update(void)
{
    int i;

    if (margp_mapping.gx_spare)
    {
        margp.gx = NAN;
        for (i = 0; i < MAX_MARGP_SENSORS &&
                 (margp_mapping.gx[i] != NULL) &&
                 (*margp_mapping.gx[i] == NAN); i++);
        if (i < MAX_MARGP_SENSORS && margp_mapping.gx[i] != NULL)
            margp.gx = *margp_mapping.gx[i];
    }
    else
    {
        // Combining sensor outputs
    }

    if (margp_mapping.gy_spare)
    {
        margp.gy = NAN;
        for (i = 0; i < MAX_MARGP_SENSORS &&
                 (margp_mapping.gy[i] != NULL) &&
                 (*margp_mapping.gy[i] == NAN); i++);
        if (i < MAX_MARGP_SENSORS && margp_mapping.gy[i] != NULL)
            margp.gy = *margp_mapping.gy[i];
    }
    else
    {
        // Combining sensor outputs
    }

    if (margp_mapping.gz_spare)
    {
        margp.gz = NAN;
        for (i = 0; i < MAX_MARGP_SENSORS &&
                 (margp_mapping.gz[i] != NULL) &&
                 (*margp_mapping.gz[i] == NAN); i++);
        if (i < MAX_MARGP_SENSORS && margp_mapping.gz[i] != NULL)
            margp.gz = *margp_mapping.gz[i];
    }
    else
    {
        // Combining sensor outputs
    }

    if (margp_mapping.ax_spare)
    {
        margp.ax = NAN;
        for (i = 0; i < MAX_MARGP_SENSORS &&
                 (margp_mapping.ax[i] != NULL) &&
                 (*margp_mapping.ax[i] == NAN); i++);
        if (i < MAX_MARGP_SENSORS && margp_mapping.ax[i] != NULL)
            margp.ax = *margp_mapping.ax[i];
    }
    else
    {
        // Combining sensor outputs
    }

    if (margp_mapping.ay_spare)
    {
        margp.ay = NAN;
        for (i = 0; i < MAX_MARGP_SENSORS &&
                 (margp_mapping.ay[i] != NULL) &&
                 (*margp_mapping.ay[i] == NAN); i++);
        if (i < MAX_MARGP_SENSORS && margp_mapping.ay[i] != NULL)
            margp.ay = *margp_mapping.ay[i];
    }
    else
    {
        // Combining sensor outputs
    }

    if (margp_mapping.az_spare)
    {
        margp.az = NAN;
        for (i = 0; i < MAX_MARGP_SENSORS &&
                 (margp_mapping.az[i] != NULL) &&
                 (*margp_mapping.az[i] == NAN); i++);
        if (i < MAX_MARGP_SENSORS && margp_mapping.az[i] != NULL)
            margp.az = *margp_mapping.az[i];
    }
    else
    {
        // Combining sensor outputs
    }

    if (margp_mapping.mx_spare)
    {
        margp.mx = NAN;
        for (i = 0; i < MAX_MARGP_SENSORS &&
                 (margp_mapping.mx[i] != NULL) &&
                 (*margp_mapping.mx[i] == NAN); i++);
        if (i < MAX_MARGP_SENSORS && margp_mapping.mx[i] != NULL)
            margp.mx = *margp_mapping.mx[i];
    }
    else
    {
        // Combining sensor outputs
    }
               
    if (margp_mapping.my_spare)
    {
        margp.my = NAN;
        for (i = 0; i < MAX_MARGP_SENSORS &&
                 (margp_mapping.my[i] != NULL) &&
                 (*margp_mapping.my[i] == NAN); i++);
        if (i < MAX_MARGP_SENSORS && margp_mapping.my[i] != NULL)
            margp.my = *margp_mapping.my[i];
    }
    else
    {
        // Combining sensor outputs
    }
               
    if (margp_mapping.mz_spare)
    {
        margp.mz = NAN;
        for (i = 0; i < MAX_MARGP_SENSORS &&
                 (margp_mapping.mz[i] != NULL) &&
                 (*margp_mapping.mz[i] == NAN); i++);
        if (i < MAX_MARGP_SENSORS && margp_mapping.mz[i] != NULL)
            margp.mz = *margp_mapping.mz[i];
    }
    else
    {
        // Combining sensor outputs
    }

    if (margp_mapping.pres_spare)
    {
        margp.pres = NAN;
        for (i = 0; i < MAX_MARGP_SENSORS &&
                 (margp_mapping.pres[i] != NULL) &&
                 (*margp_mapping.pres[i] == NAN); i++);
        if (i < MAX_MARGP_SENSORS && margp_mapping.pres[i] != NULL)
            margp.pres = *margp_mapping.pres[i];
    }
    else
    {
        // Combining sensor outputs
    }
}

