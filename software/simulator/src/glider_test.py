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
rb0 = array([0.0, 0.0, 0.0 ])
rw0 = array([0.0, 0.0, 0.00 ])

KL0 = 0.0
KL = 132.5
KD0 = 2.15
KD = 25.0
KSF0 = 0.0
KSF = -90.0
l = 0.5
V0 = 0.3

KM = array([-100.0, -100.0, -100.0])
KM0 = array([0.0, 0.0, 0.0])
KR = 1.0
KLE = 0.0
KMR = array([1.0, 0.0, 1.0])
KME = array([0.0, 0.0, 0.0])
KMA = array([0.0, 0.0, 0.0])

KDR = 0.0
KDE = 0.0
KDA = 0.0

KOmega1 = array([[   -50.0,      0.0,      0.0],
                 [     0.0,    -50.0,      0.0],
                 [     0.0,      0.0,    -50.0]])
KOmega2 = array([[   -50.0,      0.0,      0.0],
                 [     0.0,    -50.0,      0.0],
                 [     0.0,      0.0,    -50.0]])

motors = [(array([-0.0,  0.0,  0.0]), array([1.0,  0,  0]), 5.0,  0.0)]

current_velocity = array([0.0, 0.0, 0.0])

rp3 = 0.00

down_glide = True
top_depth = 10
bottom_depth = 20

model = GliderModelFull(intertia_matrix = J,
                        added_masses = Mf,
                        dry_hull_mass = Mh,
                        displacement_mass = Mfull,
                        point_mass = Mp,
                        balance_mass = Mw,
                        balance_mass_position = rw0,
                        ballast_mass_position = rb0,
                        point_mass_z_offset = rp3,
                        lift_coeff0 = KL0,
                        lift_coeff = KL,
                        drag_coeff0 = KD0,
                        drag_coeff = KD,
                        sideforce_coeff0 = KSF0,
                        sideforce_coeff = KSF,
                        rudder_sideforce_coeff = KR,
                        rudder_momentum_coeffs = KMR,
                        elevator_liftforce_coeff = KLE,
                        elevator_momentum_coeffs = KME,
                        aelerons_momentum_coeffs = KMA,
                        rudder_drag_coeff = KDR,
                        elevator_drag_coeff = KDE,
                        aelerons_drag_coeff = KDA,
                        viscous_momentum_coeffs0 = KM0,
                        viscous_momentum_coeffs = KM,
                        motors = motors,
                        damping_matrix_linear = KOmega1,
                        damping_matrix_quadratic = KOmega2,
                        current_velocity = current_velocity,
                        specific_length = l,
                        nominal_velocity = V0)

angle = -25 * (pi/180)
V = 0.3
                       
orientation = array([0.0, 0.0, 0.0])#array([0.0 * (pi/180), (angle + model.get_steady_alpha(V, angle)), 0.0])
position = array([0.0, 0.0, 0.0])
angular_velocity = array([0.0, 0.0, 0.0])
linear_velocity = array([0.0, 0.0, 0.0])#array([model.get_steady_v1(V, angle), 0.00, model.get_steady_v3(V, angle)])
point_mass_position = array([0.0, 0.0, 0.0])#array([model.get_steady_rp1(V, angle), 0.00])
point_mass_velocity = array([0.0, 0.0, 0.0])
ballast_mass_down = 1.0# 1.047
ballast_mass_up = 1.0#0.953
tmax = float(sys.argv[1])
if len(sys.argv) > 2:
    dt = float(sys.argv[2])
else:
    dt = 0.1

print "theta = %5.2f\nalpha = %5.2f\nv1 = %7.5f\nv3 = %7.5f\nrp1 = %7.5f\nmb = %7.5f" % ((angle + model.get_steady_alpha(V, angle)) / (pi/180),
                                                                                         model.get_steady_alpha(V, angle) / (pi/180),
                                                                                         model.get_steady_v1(V, angle),
                                                                                         model.get_steady_v3(V, angle),
                                                                                         model.get_steady_rp1(V, angle),
                                                                                         model.get_steady_mb(V, angle))

model.set_initial_values(0,
                         orientation = orientation,
                         position = position,
                         angular_velocity = angular_velocity,
                         linear_velocity = linear_velocity,
                         point_mass_position = array([0.0, 0.0, 0.05]),#point_mass_position,
                         point_mass_velocity = point_mass_velocity,
                         ballast_mass = 1.0)#ballast_mass)

def noise(sd, mean=0.0, n=1):
    return np.random.normal(loc=mean, scale=sd, size=n)


def get_controls(glider_step):
    global down_glide
    if (glider_step.z > bottom_depth and down_glide) or (glider_step.z < top_depth and not down_glide):
        down_glide = not down_glide
        
    if down_glide:
        mb_err = glider_step.mb - ballast_mass_down
        rp1_err = glider_step.rp1 - point_mass_position[0]
        rp2_err = glider_step.rp2 - point_mass_position[1]
    else:
        mb_err = glider_step.mb - ballast_mass_up
        rp1_err = glider_step.rp1 + point_mass_position[0]
        rp2_err = glider_step.rp2 + point_mass_position[1]

    if rp1_err != 0 and abs(rp1_err) > 0.001:
        pv1 = -rp1_err/abs(rp1_err)*0.02
    else:
        pv1 = 0
    if rp2_err != 0 and abs(rp2_err) > 0.001:
        pv2 = -rp2_err/abs(rp2_err)*0.02
    else:
        pv2 = 0
    if mb_err != 0 and abs(mb_err) > 0.00025:
        mb = -mb_err/abs(mb_err)*0.0025
    else:
        mb = 0

    return (pv1, pv2, mb, [0.2, 0.2], 0.0, 0.0, 0.0)

def W(glider_step):
    kfri = 10

    (pv1, pv2, mb, throttle, rudder_vel, aelerons_vel, elevator_vel) = get_controls(glider_step)

    w1 = pv1 - kfri*glider_step.vrp1 - kfri*glider_step.vrp1*abs(kfri*glider_step.vrp1)
    w2 = pv2 - kfri*glider_step.vrp2 - kfri*glider_step.vrp2*abs(kfri*glider_step.vrp2)
    u4 = mb
    ut = throttle
    drud = glider_step.drud + rudder_vel
    da = glider_step.da + aelerons_vel
    de = glider_step.de + elevator_vel

    return (array([w1,w2,0]), u4, ut, drud, da, de)

y_res = []
t = []

gyro_gain = 131
accel_gain = 16384
G = 9.80665

def bound(val, mi, ma):
    return min(ma, max(mi, val))

gyro_bias = array([40.0*random() - 20.0,
                   40.0*random() - 20.0,
                   40.0*random() - 20.0])

accel_bias = array([0.1*random() - 0.05,
                    0.1*random() - 0.05,
                    0.16*random() - 0.08])

while model.successful() and (tmax - model.t) > dt/2:
    y = model.next(0.1)
    y.gyro = array([bound(int((y.omega[0]/(pi/180) + noise(sd=0.005, mean=gyro_bias[0]))*gyro_gain), -32768, 32767),
                    bound(int((y.omega[1]/(pi/180) + noise(sd=0.005, mean=gyro_bias[1]))*gyro_gain), -32768, 32767),
                    bound(int((y.omega[2]/(pi/180) + noise(sd=0.005, mean=gyro_bias[2]))*gyro_gain), -32768, 32767)])
    g_vec = dot(transpose(y.R), array([0, 0, 1]))
    Fm = cross(dot(model.M, y.v) + y.pp + y.pb + y.pw, y.omega) + y.Fnet + y.Fext
    FF = Fm - y.up - (y.ub + y.uw)
    a = dot(model.Minv, FF)/G + g_vec
    y.accel = array([bound(int((a[0] + noise(sd=0.0004, mean=accel_bias[0]))*accel_gain), -32768, 32767),
                     bound(int((a[1] + noise(sd=0.0004, mean=accel_bias[1]))*accel_gain), -32768, 32767),
                     bound(int((a[2] + noise(sd=0.0004, mean=accel_bias[2]))*accel_gain), -32768, 32767)])
    (wp, u4, ut, drud, da, de) = W(y)
    model.set_controls(point_mass_accels = wp,
                       ballast_mass_change = u4,
                       thrust = ut,
                       rudder_angle = drud,
                       aelerons_angle = da,
                       elevator_angle = de)

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
      mbplt, m0plt,
      wp1plt, wp2plt, u4plt,
      ftplt, mtplt, drudplt,
      gyro1plt, gyro2plt, gyro3plt,
      accel1plt, accel2plt, accel3plt) = plt.subplots(48,1)

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
V = sqrt(v1*v1 + v2*v2 + v3*v3)
vreal1 = array(map(lambda y: y.vreal[0], y_res))
vreal2 = array(map(lambda y: y.vreal[1], y_res))
vreal3 = array(map(lambda y: y.vreal[2], y_res))
Vreal = sqrt(vreal1*vreal1 + vreal2*vreal2 + vreal3*vreal3)
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
wp1 = array(map(lambda y: y.wp[0], y_res))
wp2 = array(map(lambda y: y.wp[1], y_res))
u4 = array(map(lambda y: y.u4, y_res))
#ft = array(map(lambda y: y.FT, y_res))
#mt = array(map(lambda y: y.MT, y_res))
drud = array(map(lambda y: y.drud, y_res))
gyro1 = array(map(lambda y: y.gyro[0], y_res))
gyro2 = array(map(lambda y: y.gyro[1], y_res))
gyro3 = array(map(lambda y: y.gyro[2], y_res))
accel1 = array(map(lambda y: y.accel[0], y_res))
accel2 = array(map(lambda y: y.accel[1], y_res))
accel3 = array(map(lambda y: y.accel[2], y_res))
 
zplt.plot(t, z)
zplt.set_ylabel('depth (m)')
zplt.set_ylim([max(z), min(z)])

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
 
wp1plt.plot(t, wp1)
wp1plt.set_ylabel('$w_{p1} (m/s^2)$')
 
wp2plt.plot(t, wp2)
wp2plt.set_ylabel('$w_{p2} (m/s^2)$')

u4plt.plot(t, u4)
u4plt.set_ylabel('$u_{4} (kg/s)$')

#ftplt.plot(t, ft)
#ftplt.set_ylabel('$F_{T} (N)$')

#mtplt.plot(t, mt)
#mtplt.set_ylabel('$M_{T} (N*m)$')

drudplt.plot(t, drud / (pi / 180))
drudplt.set_ylabel('$\delta{r} (^{\circ})$')

gyro1plt.plot(t, gyro1)
gyro1plt.set_ylabel('$gyro_x$')

gyro2plt.plot(t, gyro2)
gyro2plt.set_ylabel('$gyro_y$')

gyro3plt.plot(t, gyro3)
gyro3plt.set_ylabel('$gyro_z$')

accel1plt.plot(t, accel1)
accel1plt.set_ylabel('$accel_x$')

accel2plt.plot(t, accel2)
accel2plt.set_ylabel('$accel_y$')

accel3plt.plot(t, accel3)
accel3plt.set_ylabel('$accel_z$')

plt.savefig('glider_full.png', bbox_inches='tight')

