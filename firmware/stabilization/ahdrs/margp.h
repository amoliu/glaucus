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

#ifndef _MARGP_H_
#define _MARGP_H_

#include "ch.h"

#define MAX_MARGP_SENSORS 3

struct margp_mapping
{
    // °/s
    float *gx[MAX_MARGP_SENSORS];
    bool gx_spare;
    float *gy[MAX_MARGP_SENSORS];
    bool gy_spare;
    float *gz[MAX_MARGP_SENSORS];
    bool gz_spare;
    
    // G
    float *ax[MAX_MARGP_SENSORS];
    bool ax_spare;
    float *ay[MAX_MARGP_SENSORS];
    bool ay_spare;
    float *az[MAX_MARGP_SENSORS];
    bool az_spare;
    
    // μT
    float *mx[MAX_MARGP_SENSORS];
    bool mx_spare;
    float *my[MAX_MARGP_SENSORS];
    bool my_spare;
    float *mz[MAX_MARGP_SENSORS];
    bool mz_spare;

    // Pa
    float *pres[MAX_MARGP_SENSORS];
    bool pres_spare;
};

struct margp_values
{
    // °/s
    float gx;
    float gy;
    float gz;
    
    // G
    float ax;
    float ay;
    float az;
    
    // μT
    float mx;
    float my;
    float mz;

    // Pa
    float pres;
};

extern struct margp_mapping margp_mapping;
extern struct margp_values margp;

#endif//_MARGP_H_
