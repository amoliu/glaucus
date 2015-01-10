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
#include "ahdrs.h"

#define ZETA (M_PI * (0.02f / 180.0f) * sqrt(3.0f / 4.0f))
#define BETA1 (M_PI * (.50f / 180.0f) * sqrt(3.0f / 4.0f))

static float g_bx = 0.0f;
static float g_by = 0.0f;
static float g_bz = 0.0f;

struct ahdrs_values ahdrs;

void ahdrs_init(void)
{
    ahdrs.q0 = 1.0f;
    ahdrs.q1 = 0.0f;
    ahdrs.q2 = 0.0f;
    ahdrs.q3 = 0.0f;

    ahdrs.omega_x = 0.0f;
    ahdrs.omega_y = 0.0f;
    ahdrs.omega_z = 0.0f;

    ahdrs.acc_x = 0.0f;
    ahdrs.acc_y = 0.0f;
    ahdrs.acc_z = 0.0f;

    ahdrs.flux_x = 0.0f;
    ahdrs.flux_y = 0.0f;
    ahdrs.flux_z = 0.0f;

    ahdrs.heading_x = 0.0f;
    ahdrs.heading_z = 0.0f;

    ahdrs.depth = 0.0f;
    ahdrs.vert_vel = 0.0f;
}

void ahdrs_update(float dt)
{
    // local system variables
    float norm; // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
    float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33,    // objective function Jacobian elements
        J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    float g_err_x, g_err_y, g_err_z; // estimated direction of the gyroscope error (angular)
	float deltat = dt;

    float SEq_1 = ahdrs.q0;
    float SEq_2 = ahdrs.q1;
    float SEq_3 = ahdrs.q2;
    float SEq_4 = ahdrs.q3;

    float b_x = ahdrs.heading_x;
    float b_z = ahdrs.heading_z;
	
    // axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5 * SEq_1;
    float halfSEq_2 = 0.5 * SEq_2;
    float halfSEq_3 = 0.5 * SEq_3;
    float halfSEq_4 = 0.5 * SEq_4;
    float twoSEq_1 = 2.0 * SEq_1;
    float twoSEq_2 = 2.0 * SEq_2;
    float twoSEq_3 = 2.0 * SEq_3;
    float twoSEq_4 = 2.0 * SEq_4;
    float twob_x = 2.0 * ahdrs.heading_x;
    float twob_z = 2.0 * ahdrs.heading_z;
    float twob_xSEq_1 = 2.0 * ahdrs.heading_x * SEq_1;
    float twob_xSEq_2 = 2.0 * ahdrs.heading_x * SEq_2;
    float twob_xSEq_3 = 2.0 * ahdrs.heading_x * SEq_3;
    float twob_xSEq_4 = 2.0 * ahdrs.heading_x * SEq_4;
    float twob_zSEq_1 = 2.0 * ahdrs.heading_z * SEq_1;
    float twob_zSEq_2 = 2.0 * ahdrs.heading_z * SEq_2;
    float twob_zSEq_3 = 2.0 * ahdrs.heading_z * SEq_3;
    float twob_zSEq_4 = 2.0 * ahdrs.heading_z * SEq_4;
    float SEq_1SEq_2;
    float SEq_1SEq_3 = SEq_1 * SEq_3;
    float SEq_1SEq_4;
    float SEq_2SEq_3;
    float SEq_2SEq_4 = SEq_2 * SEq_4;
    float SEq_3SEq_4;

    // normalise the accelerometer measurement
    norm = sqrt(margp.ax * margp.ax + margp.ay * margp.ay + margp.az * margp.az);
    margp.ax /= norm;
    margp.ay /= norm;
    margp.az /= norm;

    // normalise the magnetometer measurement
    norm = sqrt(margp.mx * margp.mx + margp.my * margp.my + margp.mz * margp.mz);
    margp.mx /= norm;
    margp.my /= norm;
    margp.mz /= norm;
    float twom_x = 2.0 * margp.mx;
    float twom_y = 2.0 * margp.my;
    float twom_z = 2.0 * margp.mz;

    // compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - margp.ax;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - margp.ay;
    f_3 = 1.0 - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - margp.az;
    f_4 = twob_x * (0.5 - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - margp.mx;
    f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - margp.my;
    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5 - SEq_2 * SEq_2 - SEq_3 * SEq_3) - margp.mz;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0 * SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0 * J_14or21; // negated in matrix multiplication
    J_33 = 2.0 * J_11or24; // negated in matrix multiplication
    J_41 = twob_zSEq_3; // negated in matrix multiplication
    J_42 = twob_zSEq_4;
    J_43 = 2.0 * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
    J_44 = 2.0 * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
    J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
    J_52 = twob_xSEq_3 + twob_zSEq_1;
    J_53 = twob_xSEq_2 + twob_zSEq_4;
    J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
    J_61 = twob_xSEq_3;
    J_62 = twob_xSEq_4 - 2.0 * twob_zSEq_2;
    J_63 = twob_xSEq_1 - 2.0 * twob_zSEq_3;
    J_64 = twob_xSEq_2;

    // compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 = SEqHatDot_1 / norm;
    SEqHatDot_2 = SEqHatDot_2 / norm;
    SEqHatDot_3 = SEqHatDot_3 / norm;
    SEqHatDot_4 = SEqHatDot_4 / norm;

    // compute angular estimated direction of the gyroscope error
    g_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
    g_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
    g_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;

    // compute and remove the gyroscope baises
    g_bx += g_err_x * deltat * ZETA;
    g_by += g_err_y * deltat * ZETA;
    g_bz += g_err_z * deltat * ZETA;
    ahdrs.omega_x = margp.gx - g_bx;
    ahdrs.omega_y = margp.gy - g_by;
    ahdrs.omega_z = margp.gz - g_bz;

    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * ahdrs.omega_x - halfSEq_3 * ahdrs.omega_y - halfSEq_4 * ahdrs.omega_z;
    SEqDot_omega_2 = halfSEq_1 * ahdrs.omega_x + halfSEq_3 * ahdrs.omega_z - halfSEq_4 * ahdrs.omega_y;
    SEqDot_omega_3 = halfSEq_1 * ahdrs.omega_y - halfSEq_2 * ahdrs.omega_z + halfSEq_4 * ahdrs.omega_x;
    SEqDot_omega_4 = halfSEq_1 * ahdrs.omega_z + halfSEq_2 * ahdrs.omega_y - halfSEq_3 * ahdrs.omega_x;

    // convert to °/s
    ahdrs.omega_x = 180/M_PI;
    ahdrs.omega_y = 180/M_PI;
    ahdrs.omega_z = 180/M_PI;

    // compute then integrate the estimated quaternion rate
    SEq_1 += (SEqDot_omega_1 - (BETA1 * SEqHatDot_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (BETA1 * SEqHatDot_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (BETA1 * SEqHatDot_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (BETA1 * SEqHatDot_4)) * deltat;

    // normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;

    // compute flux in the earth frame
    SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
    SEq_1SEq_3 = SEq_1 * SEq_3;
    SEq_1SEq_4 = SEq_1 * SEq_4;
    SEq_3SEq_4 = SEq_3 * SEq_4;
    SEq_2SEq_3 = SEq_2 * SEq_3;
    SEq_2SEq_4 = SEq_2 * SEq_4;
    ahdrs.flux_x = twom_x * (0.5 - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    ahdrs.flux_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5 - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
    ahdrs.flux_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5 - SEq_2 * SEq_2 - SEq_3 * SEq_3);

    ahdrs.heading_x = sqrt(ahdrs.flux_x * ahdrs.flux_x + ahdrs.flux_y * ahdrs.flux_y);
    ahdrs.heading_z = ahdrs.flux_z;

	ahdrs.q0 = SEq_1;
	ahdrs.q1 = SEq_2;
	ahdrs.q2 = SEq_3;
	ahdrs.q3 = SEq_4;

    ahdrs.acc_x = margp.ax - 2 * (ahdrs.q1*ahdrs.q3 - ahdrs.q0*ahdrs.q2);
    ahdrs.acc_y = margp.ay - 2 * (ahdrs.q0*ahdrs.q1 - ahdrs.q2*ahdrs.q3);
    ahdrs.acc_z = margp.az - (ahdrs.q0*ahdrs.q0 - ahdrs.q1*ahdrs.q1 - ahdrs.q2*ahdrs.q2 + ahdrs.q3*ahdrs.q3);

    //TODO: depth
}
