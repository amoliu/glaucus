#!/usr/bin/env python2

import matplotlib

matplotlib.use("Agg")
 
from pylab import *
import numpy as np
 
from math import atan, asin, acos, sin, cos, tan, atan2

from glider_model_full import GliderModelFull, GliderModelFullStep

import sys

rv_max = 0.001
mbv_max = 0.005
 
J1 = 4.0
J2 = 12.0
J3 = 11.0
J = array([[ J1, 0.0, 0.0],
           [0.0,  J2, 0.0],
           [0.0, 0.0,  J3]])

Mh = 20.00
Mfull = 50.0
Mp = 9.0
Mw = 20.0
Mf1 = 5.0
Mf2 = 60.0
Mf3 = 70.0
Mf = array([[Mf1, 0.0, 0.0],
            [0.0, Mf2, 0.0],
            [0.0, 0.0, Mf3]])
rb0 = array([0.0,    0.0, 0.0 ])
rw0 = array([0.0,    0.0, 0.0 ])

KL0 = 0.0
KL = 132.5
KD0 = 2.15
KD = 25.0
KMM = -100.0
KML = -100.0
KMN = -100.0
KSF0 = 0.0
KSF = -90.0

KM = array([KML, KMM, KMN])

KOmega1 = array([[   -50.0,      0.0,      0.0],
                 [     0.0,    -50.0,      0.0],
                 [     0.0,      0.0,    -50.0]])
KOmega2 = array([[   -50.0,      0.0,      0.0],
                 [     0.0,    -50.0,      0.0],
                 [     0.0,      0.0,    -50.0]])

current_velocity = array([0.5, 0.0, 0.0])

down_glide = True

model = GliderModelFull(intertia_matrix = J,
                        added_masses = Mf,
                        dry_hull_mass = Mh,
                        displacement_mass = Mfull,
                        point_mass = Mp,
                        balance_mass = Mw,
                        balance_mass_position = rw0,
                        ballast_mass_position = rb0,
                        lift_coeff = KL,
                        drag_coeff0 = KD0,
                        drag_coeff = KD,
                        sideforce_coeff0 = KSF0,
                        sideforce_coeff = KSF,
                        viscous_moment_coeffs = KM,
                        damping_matrix_linear = KOmega1,
                        damping_matrix_quadratic = KOmega2,
                        current_velocity = current_velocity)
                       
orientation = array([0.0, 0.0, 0.0])
position = array([0.0, 0.0, 0.0])
angular_velocity = array([0.0, 0.0, 0.0])
linear_velocity = array([0.0, 0.0, 0.0])
point_mass_position = array([0.0198, 0.0, 0.05])
point_mass_velocity = array([0.0, 0.0, 0.0])
ballast_mass = 1.047
tmax = float(sys.argv[1])
if len(sys.argv) > 2:
    dt = float(sys.argv[2])
else:
    dt = 0.1

model.set_initial_values(0,
                         orientation = orientation,
                         position = position,
                         angular_velocity = angular_velocity,
                         linear_velocity = linear_velocity,
                         point_mass_position = point_mass_position,
                         point_mass_velocity = point_mass_velocity,
                         ballast_mass = ballast_mass)

def W_motor(glider_step):
    kfri = 10

    global down_glide

    if down_glide and glider_step.z > 60:
        down_glide = False
    elif not down_glide and glider_step.z < 10:
        down_glide = True

    w1 = 0
    w2 = 0
    u4 = 0

    if down_glide:
        rp1_set = point_mass_position[0]
        rp2_set = point_mass_position[1]
        mb_set = ballast_mass
    else:
        rp1_set = -point_mass_position[0]
        rp2_set = -point_mass_position[1]
        mb_set = 1.0 - (ballast_mass - 1.0)

    rp1_err = glider_step.rp1 - rp1_set
    rp2_err = glider_step.rp2 - rp2_set
    mb_err = glider_step.mb - mb_set

    if abs(mb_err) > 0.00001:
        u4 = -(mb_err/abs(mb_err))*mbv_max

    if abs(rp1_err) > 0.001:
        if abs(glider_step.vrp1) < rv_max:
            w1 = -rp1_err/abs(rp1_err)*0.1
    else:
        w1 = -kfri*glider_step.vrp1

    if abs(rp2_err) > 0.001:
        if abs(glider_step.vrp2) < rv_max:
            w2 = -rp2_err/abs(rp2_err)*0.1
    else:
        w2 = -kfri*glider_step.vrp2

    return [array([w1,w2,0]), u4]

y_res = []
t = []

while model.successful() and (tmax - model.t) > dt/2:
    y = model.next(0.1)
    w = W_motor(y)
    model.set_control_accels(w)

    y_res += [y]
    t += [model.t]

    print model.t


plt.rcParams.update({'figure.figsize': (10,64)})
fig, (xplt, yplt, zplt,
      phiplt, thetaplt, psiplt,
      omega1plt, omega2plt, omega3plt,
      v1plt, v2plt, v3plt, vplt,
      vreal1plt, vreal2plt, vreal3plt, vrealplt,
      alphaplt, betaplt,
      rp1plt, rp2plt, rp3plt,
      vrp1plt, vrp2plt, vrp3plt,
      lplt, sfplt, dplt,
      m1plt, m2plt, m3plt,
      up1plt, up2plt, up3plt,
      mbplt, m0plt) = plt.subplots(36,1)

z = array(map(lambda y: y.z, y_res))
x = array(map(lambda y: y.x, y_res))
y = array(map(lambda y: y.y, y_res))
phi = array(map(lambda y: y.phi, y_res))
theta = array(map(lambda y: y.theta, y_res))
psi = array(map(lambda y: y.psi, y_res))
omega1 = array(map(lambda y: y.omega1, y_res))
omega2 = array(map(lambda y: y.omega2, y_res))
omega3 = array(map(lambda y: y.omega3, y_res))
v1 = array(map(lambda y: y.v1, y_res))
v2 = array(map(lambda y: y.v2, y_res))
v3 = array(map(lambda y: y.v3, y_res))
V = array(map(lambda y: sqrt(y.Vsq), y_res))
vreal1 = array(map(lambda y: y.vreal[0], y_res))
vreal2 = array(map(lambda y: y.vreal[1], y_res))
vreal3 = array(map(lambda y: y.vreal[2], y_res))
Vreal = array(map(lambda y: sqrt(y.Vsqreal), y_res))
alpha = array(map(lambda y: y.alpha, y_res))
beta = array(map(lambda y: y.beta, y_res))
rp1 = array(map(lambda y: y.rp1, y_res))
rp2 = array(map(lambda y: y.rp2, y_res))
rp3 = array(map(lambda y: y.rp3, y_res))
vrp1 = array(map(lambda y: y.vrp1, y_res))
vrp2 = array(map(lambda y: y.vrp2, y_res))
vrp3 = array(map(lambda y: y.vrp3, y_res))
fext = array(map(lambda y: y.Fext, y_res))
d = fext.T[0]
sf = fext.T[1]
l = fext.T[2]
text = array(map(lambda y: y.Text, y_res))
mom1 = text.T[0]
mom2 = text.T[1]
mom3 = text.T[2]
mb = array(map(lambda y: y.mb, y_res))
up1 = array(map(lambda y: y.up[0], y_res))
up2 = array(map(lambda y: y.up[1], y_res))
up3 = array(map(lambda y: y.up[2], y_res))
m0 = array(map(lambda y: y.M0, y_res))
 
zplt.plot(t, z)
zplt.set_ylabel('depth (m)')
zplt.set_ylim([65, 0])

xplt.plot(t, x)
xplt.set_ylabel('x (m)')

yplt.plot(t, y)
yplt.set_ylabel('y (m)')

phiplt.plot(t, phi / (pi/180))
phiplt.set_ylabel('$\\phi (^{\\circ})$')
 
thetaplt.plot(t, theta / (pi/180))
thetaplt.set_ylabel('$\\Theta (^{\\circ})$')
 
psiplt.plot(t, psi / (pi/180))
psiplt.set_ylabel('$\\psi (^{\\circ})$')
 
omega1plt.plot(t, omega1 / (pi/180))
omega1plt.set_ylabel('$\\Omega_1 (^{\\circ}/s)$')
 
omega2plt.plot(t, omega2 / (pi/180))
omega2plt.set_ylabel('$\\Omega_2 (^{\\circ}/s)$')
 
omega3plt.plot(t, omega3 / (pi/180))
omega3plt.set_ylabel('$\\Omega_3 (^{\\circ}/s)$')
 
v1plt.plot(t, v1)
v1plt.set_ylabel('$v_{r1} (m/s)$')
 
v2plt.plot(t, v2)
v2plt.set_ylabel('$v_{r2} (m/s)$')
 
v3plt.plot(t, v3)
v3plt.set_ylabel('$v_{r3} (m/s)$')
 
vplt.plot(t, V)
vplt.set_ylabel('$v_{r} (m/s)$')
 
vreal1plt.plot(t, vreal1)
vreal1plt.set_ylabel('$v_1 (m/s)$')
 
vreal2plt.plot(t, vreal2)
vreal2plt.set_ylabel('$v_2 (m/s)$')
 
vreal3plt.plot(t, vreal3)
vreal3plt.set_ylabel('$v_3 (m/s)$')
 
vrealplt.plot(t, Vreal)
vrealplt.set_ylabel('$v (m/s)$')
 
alphaplt.plot(t, alpha / (pi/180))
alphaplt.set_ylabel('$\\alpha (^{\circ})$')

betaplt.plot(t, beta / (pi/180))
betaplt.set_ylabel('$\\beta (^{\circ})$')
 
rp1plt.plot(t, rp1)
rp1plt.set_ylabel('$r_{P_1} (m)$')
 
rp2plt.plot(t, rp2)
rp2plt.set_ylabel('$r_{P_2} (m)$')
 
rp3plt.plot(t, rp3)
rp3plt.set_ylabel('$r_{P_3} (m)$')
 
vrp1plt.plot(t, vrp1)
vrp1plt.set_ylabel('$\dot r_{P_1} (m/s)$')
 
vrp2plt.plot(t, vrp2)
vrp2plt.set_ylabel('$\dot r_{P_2} (m/s)$')
 
vrp3plt.plot(t, vrp3)
vrp3plt.set_ylabel('$\dot r_{P_3} (m/s)$')
 
lplt.plot(t, l)
lplt.set_ylabel('$F_{L} (N)$')
 
dplt.plot(t, d)
dplt.set_ylabel('$F_{D} (N)$')
 
sfplt.plot(t, sf)
sfplt.set_ylabel('$F_{SF} (N)$')
 
m1plt.plot(t, mom1)
m1plt.set_ylabel('$M_{1} (N*m)$')
 
m2plt.plot(t, mom2)
m2plt.set_ylabel('$M_{2} (N*m)$')
 
m3plt.plot(t, mom3)
m3plt.set_ylabel('$M_{3} (N*m)$')
 
up1plt.plot(t, up1)
up1plt.set_ylabel('$F_{cp1} (N)$')
 
up2plt.plot(t, up2)
up2plt.set_ylabel('$F_{cp2} (N)$')
 
up3plt.plot(t, up3)
up3plt.set_ylabel('$F_{cp3} (N)$')
 
mbplt.plot(t, mb)
mbplt.set_ylabel('$m_b (kg)$')
 
m0plt.plot(t, m0)
m0plt.set_ylabel('$m_0 (kg)$')
 
plt.savefig('glider_full.png', bbox_inches='tight')

