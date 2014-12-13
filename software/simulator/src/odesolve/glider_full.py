#!/usr/bin/env python2

import matplotlib

matplotlib.use("Agg")
 
from pylab import *
import numpy as np
 
from scipy.integrate import ode
from math import atan, asin, acos, sin, cos, tan

J1 = 4.0
J2 = 12.0
J3 = 11.0
J = array([[ J1, 0.0, 0.0],
           [0.0,  J2, 0.0],
           [0.0, 0.0,  J3]])
Jinv = inv(J)

Mh = 39.99
Mfull = 50.0
Mp = 9.0
Mw = 0.01
Mf1 = 5.0
Mf2 = 60.0
Mf3 = 70.0
M = array([[Mh + Mf1,      0.0,      0.0],
           [     0.0, Mh + Mf2,      0.0],
           [     0.0,      0.0, Mh + Mf3]])
Minv = inv(M)

rw0 = array([0.0,    0.0, 0.0 ])
rb0 = array([0.0,    0.0, 0.0 ])
rp0 = array([0.0198, 0.00002, 0.05 ])

KL0 = 0.0
KL = 132.5
KD0 = 2.15
KD = 25.0
KMM0 = 0.0
KMM = -100.0
KML0 = 0.0
KML = -100.0
KMN0 = 0.0
KMN = -100.0
KSF0 = 0.0
KSF = 25.0

KOmega1 = array([[   -50.0,      0.0,      0.0],
                 [     0.0,    -50.0,      0.0],
                 [     0.0,      0.0,    -50.0]])
KOmega2 = array([[   -50.0,      0.0,      0.0],
                 [     0.0,    -50.0,      0.0],
                 [     0.0,      0.0,    -50.0]])

phi = 0
theta = 0#-22.9 * (pi/180)
psi = 0
q0_0 = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2)
q1_0 = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2)
q2_0 = cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2)
q3_0 = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2)
x_0 = 0.0
y_0 = 0.0
z_0 = 0.0
omega1_0 = 0.0
omega2_0 = 0.0
omega3_0 = 0.0
v1_0 = 0#0.298
v2_0 = 0.0
v3_0 = 0#0.01
rp1_0 = rp0[0]
rp2_0 = rp0[1]
rp3_0 = rp0[2]
rb1_0 = rb0[0]
rb2_0 = rb0[1]
rb3_0 = rb0[2]
rw1_0 = rw0[0]
rw2_0 = rw0[1]
rw3_0 = rw0[2]
vrp1_0 = 0.0
vrp2_0 = 0.0
vrp3_0 = 0.0
vrb1_0 = 0.0
vrb2_0 = 0.0
vrb3_0 = 0.0
vrw1_0 = 0.0
vrw2_0 = 0.0
vrw3_0 = 0.0
mb_0 = 1.000

def f_hydro(Vsq, alpha, beta):
    D = (KD0 + KD*alpha*alpha)*Vsq
    SF = (KSF0 + KSF*beta)*Vsq
    L = (KL0 + KL*alpha)*Vsq
    return array([-D, SF, -L])

def m_hydro(Vsq, alpha, beta, omega):
    return array([KML0 + KML*beta,
                  KMM0 + KMM*alpha,
                  KMN0 + KMN*beta])*Vsq + dot(KOmega1, omega) + dot(KOmega2, omega*omega)

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

def W_motor(t, yy):
    q0 = yy[0]
    q1 = yy[1]
    q2 = yy[2]
    q3 = yy[3]
    x = yy[4]
    y = yy[5]
    z = yy[6]
    omega1 = yy[7]
    omega2 = yy[8]
    omega3 = yy[9]
    v1 = yy[10]
    v2 = yy[11]
    v3 = yy[12]
    rp1 = yy[13]
    rp2 = yy[14]
    rp3 = yy[15]
    rb1 = yy[16]
    rb2 = yy[17]
    rb3 = yy[18]
    rw1 = yy[19]
    rw2 = yy[20]
    rw3 = yy[21]
    vrp1 = yy[22]
    vrp2 = yy[23]
    vrp3 = yy[24]
    vrb1 = yy[25]
    vrb2 = yy[26]
    vrb3 = yy[27]
    vrw1 = yy[28]
    vrw2 = yy[29]
    vrw3 = yy[30]
    mb = yy[31]

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

    return [array([w1,0,0]), u4]


def hat(x):
    return array([[    0, -x[2],  x[1]],
                  [ x[2],     0, -x[0]],
                  [-x[1],  x[0],     0]])


def glider_full(t, yy):
    q0 = yy[0]
    q1 = yy[1]
    q2 = yy[2]
    q3 = yy[3]
    x = yy[4]
    y = yy[5]
    z = yy[6]
    omega1 = yy[7]
    omega2 = yy[8]
    omega3 = yy[9]
    v1 = yy[10]
    v2 = yy[11]
    v3 = yy[12]
    rp1 = yy[13]
    rp2 = yy[14]
    rp3 = yy[15]
    rb1 = yy[16]
    rb2 = yy[17]
    rb3 = yy[18]
    rw1 = yy[19]
    rw2 = yy[20]
    rw3 = yy[21]
    vrp1 = yy[22]
    vrp2 = yy[23]
    vrp3 = yy[24]
    vrb1 = yy[25]
    vrb2 = yy[26]
    vrb3 = yy[27]
    vrw1 = yy[28]
    vrw2 = yy[29]
    vrw3 = yy[30]
    mb = yy[31]

    Vsq = v1*v1 + v2*v2 + v3*v3
    if v1 != 0:
        alpha = atan(v3 / v1)
    else:
        alpha = pi / 2
    if Vsq != 0:
        beta = atan(v2 / sqrt(Vsq))
    else:
        beta = 0

    R11 = q0*q0 + q1*q1 - q2*q2 - q3*q3
    R12 = 2*(q1*q2 - q0*q3)
    R13 = 2*(q1*q3 + q0*q2)
    R21 = 2*(q1*q2 + q0*q3)
    R22 = q0*q0 - q1*q1 + q2*q2 - q3*q3
    R23 = 2*(q2*q3 - q0*q1)
    R31 = 2*(q1*q3 - q0*q2)
    R32 = 2*(q2*q3 + q0*q1)
    R33 = q0*q0 - q1*q1 - q2*q2 + q3*q3

    q = array([q0, q1, q2, q3])
    dR = 0.5*array([[     0, -omega1, -omega2, -omega3],
                    [omega1,       0,  omega3, -omega2],
                    [omega2, -omega3,       0,  omega1],
                    [omega3,  omega2, -omega1,       0]])

    R = array([[R11, R12, R13],
               [R21, R22, R23],
               [R31, R32, R33]])
    RT = transpose(R)

    RWB = array([[cos(alpha)*cos(beta), -cos(alpha)*sin(beta), -sin(alpha)],
                 [           sin(beta),             cos(beta),           0],
                 [sin(alpha)*cos(beta), -sin(alpha)*sin(beta),  cos(alpha)]])

    b = array([x, y, z])

    omega = array([omega1, omega2, omega3])

    v = array([v1, v2, v3])

    rp = array([rp1, rp2, rp3])
    rb = array([rb1, rb2, rb3])
    rw = array([rw1, rw2, rw3])

    vrp = array([vrp1, vrp2, vrp3])
    vrb = array([vrb1, vrb2, vrb3])
    vrw = array([vrw1, vrw2, vrw3])

    M0 = mb + Mh + Mp + Mw - Mfull
    Fext = dot(RWB, f_hydro(Vsq, alpha, beta))
    Text = dot(RWB, m_hydro(Vsq, alpha, beta, omega))

    [wp, u4] = W_motor(t, yy)
    wb = array([0.0, 0.0, 0.0])
    ww = array([0.0, 0.0, 0.0])

    F11 = Minv - dot(dot(hat(rp), Jinv), hat(rp)) + 1.0/Mp*identity(3)
    F12 = Minv - dot(dot(hat(rp), Jinv), hat(rb))
    F13 = Minv - dot(dot(hat(rp), Jinv), hat(rw))
    F21 = Minv - dot(dot(hat(rb), Jinv), hat(rp))
    F22 = Minv - dot(dot(hat(rb), Jinv), hat(rb)) + 1.0/mb*identity(3)
    F23 = Minv - dot(dot(hat(rb), Jinv), hat(rw))
    F31 = Minv - dot(dot(hat(rw), Jinv), hat(rp))
    F32 = Minv - dot(dot(hat(rw), Jinv), hat(rb))
    F33 = Minv - dot(dot(hat(rw), Jinv), hat(rw)) + 1.0/Mw*identity(3)

    detF = inv(dot(F13, dot(F21, F32) - dot(F22, F31)) +
               dot(F12, dot(F23, F31) - dot(F21, F33)) +
               dot(F11, dot(F22, F33) - dot(F23, F32)))
    H11 = dot(detF, dot(F22, F33) - dot(F23, F32))
    H12 = dot(detF, dot(F13, F32) - dot(F12, F33))
    H13 = dot(detF, dot(F12, F23) - dot(F13, F22))
    H21 = dot(detF, dot(F23, F31) - dot(F21, F33))
    H22 = dot(detF, dot(F11, F33) - dot(F13, F31))
    H23 = dot(detF, dot(F13, F21) - dot(F11, F23))
    H31 = dot(detF, dot(F21, F32) - dot(F22, F31))
    H32 = dot(detF, dot(F12, F31) - dot(F11, F32))
    H33 = dot(detF, dot(F11, F22) - dot(F12, F21))

    k = array([0.0, 0.0, 1.0])

    pp = Mp*(v + cross(omega, rp) + vrp)
    pb = mb*(v + cross(omega, rb) + vrb)
    pw = Mw*(v + cross(omega, rw) + vrw)

    Fnet = M0*G*dot(RT, k)
#    Fin = cross(dot(M, v), omega)
#    Finp = cross(pp, omega)
#    Finb = cross(pb, omega)
#    Finw = cross(pw, omega)
    Tadded = cross(dot(M, v), v)
#    Tin = cross(dot(J, omega), omega)
#    Tp = Mp*G*dot(hat(rp), dot(RT, k))
#    Tb = mb*G*dot(hat(rb), dot(RT, k))
#    Tw = Mw*G*dot(hat(rw), dot(RT, k))

    Zf = - dot(Minv, cross(dot(M, v) + pp + pb + pw, omega) + Fnet + Fext)
    Zm = - dot(Jinv, + cross(+ dot(J, omega)
                             + dot(hat(rp), pp)
                             + dot(hat(rb), pb)
                             + dot(hat(rw), pw),
                             omega)
                     + Tadded
                     + Text
                     + cross(cross(omega, rp), pp)
                     + cross(cross(omega, rb), pb)
                     + cross(cross(omega, rw), pw)
                     + dot((Mp*hat(rp) + mb*hat(rb) + Mw*hat(rw))*G, dot(RT, k)))
    Zp = Zf - cross(omega, vrp) + cross(Zm, rp)
    Zb = Zf - cross(omega, vrb) + cross(Zm, rb)
    Zw = Zf - cross(omega, vrw) + cross(Zm, rw)

    up = dot(H11, -Zp + wp) + dot(H12, -Zb + wb) + dot(H13, -Zw + ww)
    ub = dot(H21, -Zp + wp) + dot(H22, -Zb + wb) + dot(H23, -Zw + ww)
    uw = dot(H31, -Zp + wp) + dot(H32, -Zb + wb) + dot(H33, -Zw + ww)

    Tup = dot(hat(rp), up)
    Tub = dot(hat(rb), ub)
    Tuw = dot(hat(rw), uw)

    TT = (+ cross(+ dot(J, omega)
                  + dot(hat(rp), pp)
                  + dot(hat(rb), pb)
                  + dot(hat(rw), pw),
                  omega)
          + Tadded
          + cross(cross(omega, rp), pp)
          + cross(cross(omega, rb), pb)
          + cross(cross(omega, rw), pw)
          + dot((Mp*hat(rp) + mb*hat(rb) + Mw*hat(rw))*G, dot(RT, k))
          + Text
          - dot(hat(rp), up) - (dot(hat(rb), ub) + dot(hat(rw), uw)))
         
    FF = (+ cross(dot(M, v) + pp + pb + pw, omega)
          + Fnet
          + Fext
          - up - (ub + uw))
    
    dq = dot(dR, q)
    db = dot(R, v)
    domega = dot(Jinv, TT)
    dv = dot(Minv, FF)
    drp = vrp
    drb = vrb
    drw = vrw
    dvrp = wp
    dvrb = wb
    dvrw = ww
    dmb = u4

    ret = [    dq[0],     dq[1],     dq[2],      dq[3],
               db[0],     db[1],     db[2],
           domega[0], domega[1], domega[2],
               dv[0],     dv[1],     dv[2],
              drp[0],    drp[1],    drp[2],
              drb[0],    drb[1],    drb[2],
              drw[0],    drw[1],    drw[2],
             dvrp[0],   dvrp[1],   dvrp[2],
             dvrb[0],   dvrb[1],   dvrb[2],
             dvrw[0],   dvrw[1],   dvrw[2],
                 dmb]

    return ret


def print_step(t, yy):
    q0 = yy[0]
    q1 = yy[1]
    q2 = yy[2]
    q3 = yy[3]
    x = yy[4]
    y = yy[5]
    z = yy[6]
    omega1 = yy[7]
    omega2 = yy[8]
    omega3 = yy[9]
    v1 = yy[10]
    v2 = yy[11]
    v3 = yy[12]
    rp1 = yy[13]
    rp2 = yy[14]
    rp3 = yy[15]
    rb1 = yy[16]
    rb2 = yy[17]
    rb3 = yy[18]
    rw1 = yy[19]
    rw2 = yy[20]
    rw3 = yy[21]
    vrp1 = yy[22]
    vrp2 = yy[23]
    vrp3 = yy[24]
    vrb1 = yy[25]
    vrb2 = yy[26]
    vrb3 = yy[27]
    vrw1 = yy[28]
    vrw2 = yy[29]
    vrw3 = yy[30]
    mb = yy[31]
    theta = asin(2*(q0*q2 - q3*q1))
    Vsq = v1*v1 + v2*v2 + v3*v3
    if sqrt(v1*v1 + v3*v3) > 0:
        alpha = atan(v3 / v1)
    else:
        alpha = 0
    if Vsq != 0:
        beta = atan(v2 / sqrt(Vsq))
    else:
        beta = 0

    print ("%8.4f %15e %15e %15e %15e %15e %15e %15e %15e %15e %15e %15e %15e %15e") % (t, z, x, theta, omega2, v1, v3, sqrt(v1*v1+v3*v3+v2*v2), alpha, rp1, rp3, vrp1, vrp3, mb)

    
y0 = [ q0_0,     q1_0,     q2_0,   q3_0,
        x_0,      y_0,      z_0,
   omega1_0, omega2_0, omega3_0,
       v1_0,     v2_0,     v3_0,
      rp1_0,    rp2_0,    rp3_0,
      rb1_0,    rb2_0,    rb3_0,
      rw1_0,    rw2_0,    rw3_0,
     vrp1_0,   vrp2_0,   vrp3_0,
     vrb1_0,   vrb2_0,   vrb3_0,
     vrw1_0,   vrw2_0,   vrw3_0,
       mb_0]
 
solver = ode(glider_full)
solver.set_integrator("dopri5", nsteps=500)
solver.set_initial_value(y0, 0)
 
y_res = [array(y0)]
t = [0]
print "       t               z               x           theta          omega2              v1              v3               V           alpha             rp1             rp3            vrp1            vrp3              mb"
while solver.successful() and solver.t < 1000:
    solver.integrate(solver.t + 0.1, step=100)
   
    y_res.append(solver.y)
    t.append(solver.t)
    print_step(solver.t, solver.y)

y_res = y_res[:-1]
t = t[:-1]
y_res = array(y_res).T

q0 = y_res[0]
q1 = y_res[1]
q2 = y_res[2]
q3 = y_res[3]
x = y_res[4]
y = y_res[5]
z = y_res[6]
omega1 = y_res[7]
omega2 = y_res[8]
omega3 = y_res[9]
v1 = y_res[10]
v2 = y_res[11]
v3 = y_res[12]
rp1 = y_res[13]
rp2 = y_res[14]
rp3 = y_res[15]
rb1 = y_res[16]
rb2 = y_res[17]
rb3 = y_res[18]
rw1 = y_res[19]
rw2 = y_res[20]
rw3 = y_res[21]
vrp1 = y_res[22]
vrp2 = y_res[23]
vrp3 = y_res[24]
vrb1 = y_res[25]
vrb2 = y_res[26]
vrb3 = y_res[27]
vrw1 = y_res[28]
vrw2 = y_res[29]
vrw3 = y_res[30]
mb = y_res[31]

Vsq = v1*v1 + v2*v2 + v3*v3
phi = array(map(atan, 2*(q0*q1 + q2*q3) / (1 - 2*(q1*q1 + q2*q2))))
theta = array(map(asin, 2*(q0*q2 - q3*q1)))
psi = array(map(atan, 2*(q0*q3 + q1*q2) / (1 - 2*(q2*q2 + q3*q3))))
alpha = array(map(atan, v3 / v1))
beta = array(map(atan, v2 / sqrt(Vsq)))
RWB = array([[array(map(cos, alpha))*array(map(cos, beta)), -array(map(cos, alpha))*array(map(sin, beta)), -array(map(sin, alpha))],
             [           array(map(sin, beta)),             array(map(cos, beta)),           0],
             [array(map(sin, alpha))*array(map(cos, beta)), -array(map(sin, alpha))*array(map(sin, beta)),  array(map(cos, alpha))]])


fext = f_hydro(Vsq, alpha, beta)
text = m_hydro(Vsq, alpha, beta, array([omega1, omega2, omega3]))

d = (RWB[0,0]*fext[0] + RWB[0,1]*fext[1] + RWB[0,2]*fext[2])
sf = (RWB[1,0]*fext[0] + RWB[1,1]*fext[1] + RWB[1,2]*fext[2])
l = (RWB[2,0]*fext[0] + RWB[2,1]*fext[1] + RWB[2,2]*fext[2])

mom1 = (RWB[0,0]*text[0] + RWB[0,1]*text[1] + RWB[0,2]*text[2])
mom2 = (RWB[1,0]*text[0] + RWB[1,1]*text[1] + RWB[1,2]*text[2])
mom3 = (RWB[2,0]*text[0] + RWB[2,1]*text[1] + RWB[2,2]*text[2])

m0 = mb + Mh + Mp + Mw - Mfull

plt.rcParams.update({'figure.figsize': (10,48)})
fig, (xplt, yplt, zplt,
      phiplt, thetaplt, psiplt,
      omega1plt, omega2plt, omega3plt,
      v1plt, v2plt, v3plt, vplt,
      alphaplt, betaplt,
      rp1plt, rp2plt, rp3plt,
      vrp1plt, vrp2plt, vrp3plt,
      lplt, sfplt, dplt,
      m1plt, m2plt, m3plt,
      mbplt, m0plt) = plt.subplots(29,1)
 
zplt.plot(t, z)
zplt.set_ylabel('depth (m)')
zplt.set_ylim([65, 0])

xplt.plot(t, x)
xplt.set_ylabel('x (m)')
#xplt.set_ylim([0, 200])

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
v1plt.set_ylabel('$v_1 (m/s)$')
 
v2plt.plot(t, v2)
v2plt.set_ylabel('$v_2 (m/s)$')
 
v3plt.plot(t, v3)
v3plt.set_ylabel('$v_3 (m/s)$')
 
vplt.plot(t, sqrt(Vsq))
vplt.set_ylabel('v (m/s)')
 
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
 
mbplt.plot(t, mb)
mbplt.set_ylabel('$m_b (kg)$')
 
m0plt.plot(t, m0)
m0plt.set_ylabel('$m_0 (kg)$')
 
plt.savefig('glider_full.png')

