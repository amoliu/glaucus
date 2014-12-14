#!/usr/bin/env python2

from pylab import *
import numpy as np
#from numba import jit
 
from scipy.integrate import ode
from math import atan, asin, acos, sin, cos, tan, atan2

G = 9.80665
 
def hat(x):
    return array([[    0, -x[2],  x[1]],
                  [ x[2],     0, -x[0]],
                  [-x[1],  x[0],     0]])

class GliderModelFullStep:
    def __init__(self,
                 glider_model,
                 t,
                 yy):
        self.t = t
        self.q0 = yy[0]
        self.q1 = yy[1]
        self.q2 = yy[2]
        self.q3 = yy[3]
        self.x = yy[4]
        self.y = yy[5]
        self.z = yy[6]
        self.omega1 = yy[7]
        self.omega2 = yy[8]
        self.omega3 = yy[9]
        self.v1 = yy[10]
        self.v2 = yy[11]
        self.v3 = yy[12]
        self.rp1 = yy[13]
        self.rp2 = yy[14]
        self.rp3 = yy[15]
        self.rb1 = yy[16]
        self.rb2 = yy[17]
        self.rb3 = yy[18]
        self.rw1 = yy[19]
        self.rw2 = yy[20]
        self.rw3 = yy[21]
        self.vrp1 = yy[22]
        self.vrp2 = yy[23]
        self.vrp3 = yy[24]
        self.vrb1 = yy[25]
        self.vrb2 = yy[26]
        self.vrb3 = yy[27]
        self.vrw1 = yy[28]
        self.vrw2 = yy[29]
        self.vrw3 = yy[30]
        self.mb = yy[31]
        self.Vsq = self.v1*self.v1 + self.v2*self.v2 + self.v3*self.v3
        self.v = array([self.v1, self.v2, self.v3])

        if self.v1 != 0:
            self.alpha = atan(self.v3 / self.v1)
        else:
            self.alpha = pi / 2
        if (self.Vsq) != 0:
            self.beta = asin(self.v2 / sqrt(self.Vsq))
        else:
            self.beta = 0
        self.q = array([self.q0, self.q1, self.q2, self.q3])
        self.phi = atan(2*(self.q0*self.q1 + self.q2*self.q3) / (1 - 2*(self.q1*self.q1 + self.q2*self.q2)))
        self.theta = asin(2*(self.q0*self.q2 - self.q3*self.q1))
        self.psi = atan2(2*(self.q1*self.q2 + self.q0*self.q3), self.q0*self.q0 + self.q1*self.q1 - self.q2*self.q2 - self.q3*self.q3)
        self.RWB = array([[cos(self.alpha)*cos(self.beta), -cos(self.alpha)*sin(self.beta), -sin(self.alpha)],
                          [                sin(self.beta),                  cos(self.beta),                0],
                          [sin(self.alpha)*cos(self.beta), -sin(self.alpha)*sin(self.beta),  cos(self.alpha)]])

        R11 = self.q0*self.q0 + self.q1*self.q1 - self.q2*self.q2 - self.q3*self.q3
        R12 = 2*(self.q1*self.q2 - self.q0*self.q3)
        R13 = 2*(self.q1*self.q3 + self.q0*self.q2)
        R21 = 2*(self.q1*self.q2 + self.q0*self.q3)
        R22 = self.q0*self.q0 - self.q1*self.q1 + self.q2*self.q2 - self.q3*self.q3
        R23 = 2*(self.q2*self.q3 - self.q0*self.q1)
        R31 = 2*(self.q1*self.q3 - self.q0*self.q2)
        R32 = 2*(self.q2*self.q3 + self.q0*self.q1)
        R33 = self.q0*self.q0 - self.q1*self.q1 - self.q2*self.q2 + self.q3*self.q3

        self.R = array([[R11, R12, R13],
                        [R21, R22, R23],
                        [R31, R32, R33]])
        RT = transpose(self.R)

        self.vreal = dot(self.R, self.v) + glider_model.vc
        self.Vsqreal = self.vreal[0]*self.vreal[0] + self.vreal[1]*self.vreal[1] + self.vreal[2]*self.vreal[2]

        self.b = array([self.x, self.y, self.z])

        self.omega = array([self.omega1, self.omega2, self.omega3])

        self.v = array([self.v1, self.v2, self.v3])

        self.rp = array([self.rp1, self.rp2, self.rp3])
        self.rb = array([self.rb1, self.rb2, self.rb3])
        self.rw = array([self.rw1, self.rw2, self.rw3])

        self.vrp = array([self.vrp1, self.vrp2, self.vrp3])
        self.vrb = array([self.vrb1, self.vrb2, self.vrb3])
        self.vrw = array([self.vrw1, self.vrw2, self.vrw3])

        f_hydro = glider_model.f_hydro
        m_hydro = glider_model.m_hydro
        self.M0 = self.mb + glider_model.Mh + glider_model.Mp + glider_model.Mw - glider_model.Mfull
        self.Fext = dot(self.RWB, f_hydro(self.Vsq, self.alpha, self.beta))
        self.Text = dot(self.RWB, m_hydro(self.Vsq, self.alpha, self.beta, self.omega))
        F11 = glider_model.Minv - dot(dot(hat(self.rp), glider_model.Jinv), hat(self.rp)) + 1.0/glider_model.Mp*identity(3)
        F12 = glider_model.Minv - dot(dot(hat(self.rp), glider_model.Jinv), hat(self.rb))
        F13 = glider_model.Minv - dot(dot(hat(self.rp), glider_model.Jinv), hat(self.rw))
        F21 = glider_model.Minv - dot(dot(hat(self.rb), glider_model.Jinv), hat(self.rp))
        F22 = glider_model.Minv - dot(dot(hat(self.rb), glider_model.Jinv), hat(self.rb)) + 1.0/self.mb*identity(3)
        F23 = glider_model.Minv - dot(dot(hat(self.rb), glider_model.Jinv), hat(self.rw))
        F31 = glider_model.Minv - dot(dot(hat(self.rw), glider_model.Jinv), hat(self.rp))
        F32 = glider_model.Minv - dot(dot(hat(self.rw), glider_model.Jinv), hat(self.rb))
        F33 = glider_model.Minv - dot(dot(hat(self.rw), glider_model.Jinv), hat(self.rw)) + 1.0/glider_model.Mw*identity(3)

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

        self.pp = glider_model.Mp*(self.v + cross(self.omega, self.rp) + self.vrp)
        self.pb = self.mb*(self.v + cross(self.omega, self.rb) + self.vrb)
        self.pw = glider_model.Mw*(self.v + cross(self.omega, self.rw) + self.vrw)

        self.Fnet = self.M0*G*dot(RT, k)
        self.Fin = cross(dot(glider_model.M, self.v), self.omega)
        self.Finp = cross(self.pp, self.omega)
        self.Finb = cross(self.pb, self.omega)
        self.Finw = cross(self.pw, self.omega)
        self.Tadded = cross(dot(glider_model.M, self.v), self.v)
        self.Tin = cross(dot(glider_model.J, self.omega), self.omega)
        self.Tp = glider_model.Mp*G*dot(hat(self.rp), dot(RT, k))
        self.Tb = self.mb*G*dot(hat(self.rb), dot(RT, k))
        self.Tw = glider_model.Mw*G*dot(hat(self.rw), dot(RT, k))

        Zf = - dot(glider_model.Minv, cross(dot(glider_model.M, self.v) + self.pp + self.pb + self.pw, self.omega) + self.Fnet + self.Fext)
        Zm = - dot(glider_model.Jinv, + cross(+ dot(glider_model.J, self.omega)
                                              + dot(hat(self.rp), self.pp)
                                              + dot(hat(self.rb), self.pb)
                                              + dot(hat(self.rw), self.pw),
                                              self.omega)
                   + self.Tadded
                   + self.Text
                   + cross(cross(self.omega, self.rp), self.pp)
                   + cross(cross(self.omega, self.rb), self.pb)
                   + cross(cross(self.omega, self.rw), self.pw)
                   + dot((glider_model.Mp*hat(self.rp) + self.mb*hat(self.rb) + glider_model.Mw*hat(self.rw))*G, dot(RT, k)))
        Zp = Zf - cross(self.omega, self.vrp) + cross(Zm, self.rp)
        Zb = Zf - cross(self.omega, self.vrb) + cross(Zm, self.rb)
        Zw = Zf - cross(self.omega, self.vrw) + cross(Zm, self.rw)

        self.up = dot(H11, -Zp + glider_model.wp) + dot(H12, -Zb + glider_model.wb) + dot(H13, -Zw + glider_model.ww)
        self.ub = dot(H21, -Zp + glider_model.wp) + dot(H22, -Zb + glider_model.wb) + dot(H23, -Zw + glider_model.ww)
        self.uw = dot(H31, -Zp + glider_model.wp) + dot(H32, -Zb + glider_model.wb) + dot(H33, -Zw + glider_model.ww)
        self.wp = glider_model.wp
        self.wb = glider_model.wb
        self.ww = glider_model.ww
        self.u4 = glider_model.u4

        self.Tup = dot(hat(self.rp), self.up)
        self.Tub = dot(hat(self.rb), self.ub)
        self.Tuw = dot(hat(self.rw), self.uw)


class GliderModelFull:
    def __init__(self,
                 intertia_matrix,
                 added_masses,
                 dry_hull_mass,
                 displacement_mass,
                 point_mass,
                 balance_mass,
                 balance_mass_position,
                 ballast_mass_position,
                 lift_coeff,
                 drag_coeff0,
                 drag_coeff,
                 sideforce_coeff0,
                 sideforce_coeff,
                 viscous_moment_coeffs,
                 damping_matrix_linear,
                 damping_matrix_quadratic,
                 current_velocity):
        self.J = intertia_matrix
        J = self.J
        self.Jinv = inv(J)
        Jinv = self.Jinv
        self.M = added_masses + dry_hull_mass * identity(3)
        M = self.M 
        self.Minv = inv(M)
        Minv = self.Minv 
        self.Mfull = displacement_mass
        Mfull = self.Mfull 
        self.Mh = dry_hull_mass
        Mh = self.Mh 
        self.Mp = point_mass
        Mp = self.Mp 
        self.Mw = balance_mass
        Mw = self.Mw 
        self.rw0 = balance_mass_position
        rw0 = self.rw0 
        self.rb0 = ballast_mass_position
        rb0 = self.rb0 
        self.KL = lift_coeff
        KL = self.KL 
        self.KD0 = drag_coeff0
        KD0 = self.KD0 
        self.KD = drag_coeff
        KD = self.KD 
        self.KSF0 = sideforce_coeff0
        KSF0 = self.KSF0 
        self.KSF = sideforce_coeff
        KSF = self.KSF 
        self.KM = viscous_moment_coeffs
        KM = self.KM 
        self.KOmega1 = damping_matrix_linear
        KOmega1 = self.KOmega1 
        self.KOmega2 = damping_matrix_quadratic
        KOmega2 = self.KOmega2 
        self.vc = current_velocity
        vc = self.vc

        def f_hydro(Vsq, alpha, beta):
            D = (KD0 + KD*alpha*alpha)*Vsq
            SF = (KSF0 + KSF*beta)*Vsq
            L = (KL*alpha)*Vsq
            return array([-D, SF, -L])

        self.f_hydro = f_hydro

        def m_hydro(Vsq, alpha, beta, omega):
            return (KM*array([beta, alpha, beta])*Vsq +
                    dot(KOmega1, omega) +
                    dot(KOmega2, omega*omega))

        self.m_hydro = m_hydro

        self.wp = array([0.0, 0.0, 0.0])
        self.wb = array([0.0, 0.0, 0.0])
        self.ww = array([0.0, 0.0, 0.0])
        self.u4 = 0

        def get_accels(self):
            return [self.wp,
                    self.wb,
                    self.ww,
                    self.u4]

        def get_vc(self):
            return self.vc

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

            Vsq = v1*v1 + v2*v2 + v3*v3
            if v1 != 0:
                alpha = atan(v3 / v1)
            else:
                alpha = pi / 2
            if (Vsq) != 0:
                beta = asin(v2 / sqrt(Vsq))
            else:
                beta = 0

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

            [wp, wb, ww, u4] = get_accels(self)
            vc = get_vc(self)

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
            Tadded = cross(dot(M, v), v)

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
            db = dot(R, v) + vc
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

        solver = ode(glider_full)
        solver.set_integrator("dopri5", nsteps=500)
        self.solver = solver

    def set_initial_values(self,
                           t0,
                           orientation,
                           position,
                           angular_velocity,
                           linear_velocity,
                           point_mass_position,
                           point_mass_velocity,
                           ballast_mass):
        phi = orientation[0]
        theta = orientation[1]
        psi = orientation[2]
        q0_0 = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2)
        q1_0 = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2)
        q2_0 = cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2)
        q3_0 = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2)

        y0 = [q0_0,             q1_0,                   q2_0,                   q3_0,
                         position[0],            position[1],            position[2],
                 angular_velocity[0],    angular_velocity[1],    angular_velocity[2],
                  linear_velocity[0],     linear_velocity[1],     linear_velocity[2],
              point_mass_position[0], point_mass_position[1], point_mass_position[2],
                         self.rb0[0],            self.rb0[1],            self.rb0[2],
                         self.rw0[0],            self.rw0[1],            self.rw0[2],
              point_mass_velocity[0], point_mass_velocity[1], point_mass_velocity[2],
                                 0.0,                    0.0,                    0.0,
                                 0.0,                    0.0,                    0.0,
                        ballast_mass]

        self.t = t0
        self.solver.set_initial_value(y0, t0)

    def next(self, dt):
        self.solver.integrate(self.solver.t + dt)

        self.t = self.solver.t
   
        return GliderModelFullStep(self, self.solver.t, self.solver.y)

    def successful(self):
        return self.solver.successful()

    def set_control_accels(self, w):
        self.wp = w[0]
        self.u4 = w[1]
        self.wb = array([0,0,0])
        self.ww = array([0,0,0])

    def set_current(self, current_velocity):
        self.vc = current_velocity
