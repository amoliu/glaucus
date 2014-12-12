#!/usr/bin/env python2
import matplotlib

matplotlib.use("Agg")
 
from pylab import *
import numpy as np
 
from scipy.integrate import ode
from math import atan
 
J1 = 4.0
J2 = 12.0
J3 = 11.0
Mf1 = 5.0
Mf2 = 60.0
Mf3 = 70.0
Mh = 40.0
Mfull = 50.0
Mp = 9.0
KL0 = 0.0
KL = 132.5
KD0 = 2.15
KD = 25.0
KM0 = 0.0
KM = -100.0
KOmega21 = -50
KOmega22 = 0.0#-50
G = 9.80665
rv_max = 0.001
mbv_max = 0.005
 
dt_pid = 0.1
t_pid = 0
eoo1_pid = 0
eo1_pid = 0
ie1_pid = 0
w1_pid = 0
u4_pid = 0

down_glide = True

def U(t, y):
    global t_pid
    global ie1_pid
    global ie3_pid
    global eo1_pid
    global eo3_pid

    if t_pid == t:
        return [0,0,0,0]
    else:
        dt = t - t_pid
        t_pid = t
   
    x = y[0]
    z = y[1]
    theta = y[2]
    omega2 = y[3]
    v1 = y[4]
    v3 = y[5]
    rp1 = y[6]
    rp3 = y[7]
    pp1 = y[8]
    pp3 = y[9]
    mb = y[10]

    k = 1
    ti = dt
    td = dt
    print dt

    u1 = 0
    u2 = 0
    u3 = 0
    u4 = 0

    if t < 250:
        rp1_set = 0.0198
        rp3_set = 0.05
    else:
        rp1_set = -0.0198
        rp3_set = 0.05

    rp1_err = rp1 - rp1_set
    rp3_err = rp3 - rp3_set

    ie1_pid += (rp1_err + eo1_pid) / 2.0
    ie3_pid += (rp3_err + eo3_pid) / 2.0
    eo1_pid = rp1_err
    eo3_pid = rp3_err

    u1 = k*(rp1_err + (1.0/ti)*ie1_pid + td*(rp1_err - eo1_pid))
    u3 = k*(rp3_err + (1.0/ti)*ie3_pid + td*(rp3_err - eo3_pid))
   
#    if t > 250:

    return [u1,u2,u3,u4]

def W(y, t):
    x = y[0]
    z = y[1]
    theta = y[2]
    omega2 = y[3]
    v1 = y[4]
    v3 = y[5]
    rp1 = y[6]
    rp3 = y[7]
    vrp1 = y[8]
    vrp3 = y[9]
    mb = y[10]
    krp = -2
    kri = -0.001
    krd = -100
    km = 0.01

    global eoo1_pid
    global eo1_pid
    global ie1_pid
    global w1_pid
    global u4_pid
    global t_pid

    if t % 500 < 250:
        rp1_set = 0.0198
        mb_set = 1.047
    else:
        rp1_set = -0.0198
        mb_set = 0.953

    if (t - t_pid) < dt_pid:
        return [w1_pid, 0, 0, u4_pid]

    if (t % 500 < 250 and t_pid % 500 > 250) or (t % 500 > 250 and t_pid % 500 < 250):
       eo1_pid = 0
       eoo1_pid = 0
       ie1_pid = 0 

    t_pid = t

    w1 = 0
    u4 = 0

    rp1_err = rp1 - rp1_set
    mb_err = mb - mb_set
    ie1_pid += (eo1_pid + rp1_err) / 2.0
    de1_pid = (rp1_err - eo1_pid)

    w1 = krp*rp1_err + kri*ie1_pid + krd*de1_pid
#    if rp1_err != 0:
#        print rp1_err, ie1_pid, de1_pid, w1

    eoo1_pid = eo1_pid
    eo1_pid = rp1_err

    if abs(w1) > 0.1:
        w1 = w1/abs(w1) * 0.1

    if abs(mb_err) > 0.001:
        u4 = -km*(mb_err/abs(mb_err))

    w1_pid = w1
    u4_pid = u4

    return [w1_pid,0,0,u4_pid]
       
def W_motor(y, t):
    x = y[0]
    z = y[1]
    theta = y[2]
    omega2 = y[3]
    v1 = y[4]
    v3 = y[5]
    rp1 = y[6]
    rp3 = y[7]
    vrp1 = y[8]
    vrp3 = y[9]
    mb = y[10]
    kfri = 10

    global down_glide

    if down_glide and z > 60:
        down_glide = False
    elif not down_glide and z < 10:
        down_glide = True

    w1 = 0
    u4 = 0

    if down_glide:
        rp1_set = 0.0198
        mb_set = 1.047
    else:
        rp1_set = -0.0198
        mb_set = 0.953

    rp1_err = rp1 - rp1_set
    mb_err = mb - mb_set

    if abs(mb_err) > 0.001:
        u4 = -(mb_err/abs(mb_err))*mbv_max

    if abs(rp1_err) > 0.001:
        if abs(vrp1) < rv_max:
            w1 = -rp1_err/abs(rp1_err)*0.1
    else:
        w1 = -kfri*vrp1

    return [0,0,0,0]
    return [w1,0,0,u4]


def glider_longitudal(t, y):
    x = y[0]
    z = y[1]
    theta = y[2]
    omega2 = y[3]
    v1 = y[4]
    v3 = y[5]
    rp1 = y[6]
    rp3 = y[7]
    pp1 = y[8]
    pp3 = y[9]
    mb = y[10]
   
    if v1 != 0 and v3 != 0:
        alpha = atan(v3/v1)
        sin_alpha = v3 / sqrt(v3*v3 + v1*v1)
        cos_alpha = v1 / sqrt(v3*v3 + v1*v1)
    else:
        alpha = pi/2
        sin_alpha = 0
        cos_alpha = 1
    M1 = mb + Mh + Mf1
    M2 = mb + Mh + Mf2
    M3 = mb + Mh + Mf3
    M0 = mb + Mh + Mp - Mfull
    D = (KD0 + KD*alpha*alpha)*(v1*v1 + v3*v3)
    L = (KL0 + KL*alpha)*(v1*v1 + v3*v3)
    MDL = (KM0 + KM*alpha)*(v1*v1 + v3*v3) + KOmega21*omega2 + KOmega22*omega2*omega2
    #print (Mf3 - Mf1)*v1*v3, (rp1*pp1 + rp3*pp3)*omega2, Mp*G*(rp1*cos(theta) + rp3*sin(theta)), MDL
    #print (M3 - M1)*v1*v3 - (rp1*pp1 + rp3*pp3)*omega2 - Mp*G*(rp1*cos(theta) + rp3*sin(theta)) + MDL
    [u1,_,u3,u4] = U(t,y)
 
    dx = v1*cos(theta) + v3*sin(theta)
    dz = -v1*sin(theta) + v3*cos(theta)
    dtheta = omega2
    domega2 = 1.0/J2*((M3 - M1)*v1*v3 - (rp1*pp1 + rp3*pp3)*omega2 - Mp*G*(rp1*cos(theta) + rp3*sin(theta)) + MDL - rp3*u1 + rp1*u3)
    dv1 = 1.0/M1*(-M3*v3*omega2 - pp3*omega2 - M0*G*sin(theta) + L*sin_alpha - D*cos_alpha - u1)
    dv3 = 1.0/M3*(M1*v1*omega2 + pp1*omega2 + M0*G*cos(theta) - L*cos_alpha - D*sin_alpha - u3)
    drp1 = 1.0/Mp*pp1 - v1 - rp3*omega2
    drp3 = 1.0/Mp*pp3 - v3 + rp1*omega2
    dpp1 = u1
    dpp3 = u3
    dmb = u4
   
    return [dx,dz,dtheta,domega2,dv1,dv3,drp1,drp3,dpp1,dpp3,dmb]

def glider_longitudal_accel(t, y):
    x = y[0]
    z = y[1]
    theta = y[2]
    omega2 = y[3]
    v1 = y[4]
    v3 = y[5]
    rp1 = y[6]
    rp3 = y[7]
    vrp1 = y[8]
    vrp3 = y[9]
    mb = y[10]
   
    if v1 != 0 and v3 != 0:
        alpha = atan(v3/v1)
        sin_alpha = v3 / sqrt(v3*v3 + v1*v1)
        cos_alpha = v1 / sqrt(v3*v3 + v1*v1)
    else:
        alpha = 0
        sin_alpha = 0
        cos_alpha = 1

    M1 = mb + Mh + Mf1
    M2 = mb + Mh + Mf2
    M3 = mb + Mh + Mf3
    M0 = mb + Mh + Mp - Mfull

    D = (KD0 + KD*alpha*alpha)*(v1*v1 + v3*v3)
    L = (KL0 + KL*alpha)*(v1*v1 + v3*v3)
    MDL = (KM0 + KM*alpha)*(v1*v1 + v3*v3) + KOmega21*omega2 + KOmega22*omega2*omega2

    a1 = 1.0/Mp + 1.0/M1
    a3 = 1.0/Mp + 1.0/M3
    b1 = rp1*rp1/J2
    b3 = rp3*rp3/J3
    c = rp1*rp3/J2
    d1 = a1 + b1*a1 + b3
    d3 = a3 + b3*a3 + b1

    F = (a1 + b3)*(a3 + b1) - c*c#det(array([[a1 + b3, -c], [-c, a3 + b1]]))
    X1 = -M3*v3*omega2 - Mp*(v3 + vrp3 - rp1*omega2)*omega2 - M0*G*sin(theta) + L*sin_alpha - D*cos_alpha
    X3 = M1*v1*omega2 + Mp*(v1 + vrp1 + rp3*omega2)*omega2 + M0*G*cos(theta) - L*cos_alpha - D*sin_alpha
    Y = (M3 - M1)*v1*v3 - Mp*G*(rp1*cos(theta) + rp3*sin(theta)) + MDL

    [w1,_,w3,u4] = W_motor(y, t)
 
    dx = v1*cos(theta) + v3*sin(theta)
    dz = -v1*sin(theta) + v3*cos(theta)
    dtheta = omega2
    domega2 = 1.0/(J2 * F) * (a1*a3*Y - rp3/M1*a3*X1 + rp1/M3*a1*X3 - rp1*a1*(omega2*vrp1 - w3) - rp3*a3*(omega2*vrp3 - w1))
    dv1 = 1.0/(M1*F)*(-rp3/J2*a3*Y + d3/Mp*X1 - c/M3*X3 + c*(omega2*vrp1 - w3) - (a3 + b1)*(omega2*vrp3 + w1))
    dv3 = 1.0/(M3*F)*(rp1/J2*a1*Y - c/M1*X1 + d1/Mp*X3 + (a1 + b3)*(omega2*vrp1 - w3) - c*(omega2*vrp3 + w1))
    drp1 = vrp1
    drp3 = vrp3
    dvp1 = w1
    dvp3 = w3
    dmb = u4
   
    return [dx,dz,dtheta,domega2,dv1,dv3,drp1,drp3,dvp1,dvp3,dmb]

def print_step(t, y):
    x = y[0]
    z = y[1]
    theta = y[2]
    omega2 = y[3]
    v1 = y[4]
    v3 = y[5]
    rp1 = y[6]
    rp3 = y[7]
    vrp1 = y[8]
    vrp3 = y[9]
    mb = y[10]
    if v1 != 0 and v3 != 0:
        alpha = atan(v3/v1)
    else:
        alpha = pi/2
    print ("%8.4f %15e %15e %15e %15e %15e %15e %15e %15e %15e %15e %15e %15e %15e") % (t, z, x, theta, omega2, v1, v3, sqrt(v1*v1+v3*v3), alpha, rp1, rp3, vrp1, vrp3, mb)
    
 
x0 = 0
z0 = 0
theta0 = 0#-22.9773 * pi/180.0
omega20 = 0.000
v10 = 0.0#0.3#0.0002998130856646799
v30 = 0.0#0.01#0.000010588374012251428
rp10 = 0.0#198#-0.0013706169093920316 # -0.0011617533285801819 #0.02
rp30 = 0.05#0.0 # 0.00049259309803055702 #0.05
vp10 = 0
vp30 = 0
mb0 = 1.047#3652929691557
y0 = [x0,z0,theta0,omega20,v10,v30,rp10,rp30,vp10,vp30,mb0]
 
solver = ode(glider_longitudal_accel)
solver.set_integrator("dopri5")
solver.set_initial_value(y0, 0)
 
y_res = [y0]
t = [0]
print "       t               z               x           theta          omega2              v1              v3               V           alpha             rp1             rp3            vrp1            vrp3              mb"
while solver.successful() and solver.t < 10:
    solver.integrate(solver.t + 0.1, step=200)
   
    y_res.append(solver.y)
    t.append(solver.t)
    print_step(solver.t, solver.y)
   
y_res = array(y_res)
x = y_res.T[0]
z = y_res.T[1]
theta = y_res.T[2]
omega2 = y_res.T[3]
v1 = y_res.T[4]
v3 = y_res.T[5]
rp1 = y_res.T[6]
rp3 = y_res.T[7]
vp1 = y_res.T[8]
vp3 = y_res.T[9]
mb = y_res.T[10]
 
plt.rcParams.update({'figure.figsize': (8,20)})
fig, (zplt, xplt, thetaplt, omega2plt, v1plt, v3plt, vplt, alphaplt, rp1plt, vp1plt, mbplt) = plt.subplots(11,1)
 
zplt.plot(t, z)
zplt.set_ylabel('depth (m)')
#zplt.set_ylim([65, 0])
 
xplt.plot(t, x)
xplt.set_ylabel('x (m)')
#xplt.set_ylim([0, 200])
 
thetaplt.plot(t, theta / (pi/180))
thetaplt.set_ylabel('$\Theta (^{\circ})$')
#thetaplt.set_ylim([-30, 30])
 
omega2plt.plot(t, omega2 / (pi/180))
omega2plt.set_ylabel('$\Omega_2 (^{\circ}/s)$')
#omega2plt.set_ylim([-5, 5])
 
v1plt.plot(t, v1)
v1plt.set_ylabel('$v_1 (m/s)$')
 
v3plt.plot(t, v3)
v3plt.set_ylabel('$v_3 (m/s)$')
 
vplt.plot(t, sqrt(v1*v1 + v3*v3))
vplt.set_ylabel('v (m/s)')
 
alphaplt.plot(t, array(map(atan, v3/sqrt(v1*v1 + v3*v3))) / (pi/180))
alphaplt.set_ylabel('$\\alpha (^{\circ})$')
#alphaplt.set_ylim([-5, 5])
 
rp1plt.plot(t, rp1)
rp1plt.set_ylabel('$r_{P_1} (m)$')
 
vp1plt.plot(t, vp1)
vp1plt.set_ylabel('$\dot r_{P_1} (m/s)$')
 
mbplt.plot(t, mb)
mbplt.set_ylabel('$m_b (kg)$')
 
plt.savefig('glider_longitudal.png')
