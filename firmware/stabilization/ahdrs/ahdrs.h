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

#ifndef _AHDRS_H_
#define _AHDRS_H_

#include "ch.h"

struct ahdrs_values
{
    float q0;
    float q1;
    float q2;
    float q3;

    float omega_x;
    float omega_y;
    float omega_z;

    float acc_x;
    float acc_y;
    float acc_z;

    float flux_x;
    float flux_y;
    float flux_z;

    float heading_x;
    float heading_z;

    float depth;
    float vert_vel;
};

extern struct ahdrs_values ahdrs;

extern void ahdrs_init(void);

extern void ahdrs_update(float);

#endif//_AHDRS_H_
