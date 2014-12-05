#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#define G 9.80665

void longitudal_glider(double *res, double t, double *y, int l, void *arg)
{
#define x       y[0]
#define z       y[1]
#define theta   y[2]
#define omega2  y[3]
#define v1      y[4]
#define v3      y[5]
#define rP1     y[6]
#define rP3     y[7]
#define PP1     y[8]
#define PP2     y[9]
#define mb      y[10]
#define dx      res[0]
#define dz      res[1]
#define dtheta  res[2]
#define domega2 res[3]
#define dv1     res[4]
#define dv3     res[5]
#define drP1    res[6]
#define drP3    res[7]
#define dPP1    res[8]
#define dPP2    res[9]
#define dmb     res[10]
    assert(l == 11);
    dx = v1 * cos(theta) + v3 * sin(theta);
    dz = v1 * sin(theta) + v3 * cos(theta);
    dtheta = omega2;
    domega2 = 1 / j2 * ((m3 - m1)*v1*v3 - (rP1*PP1 + rP3*PP3)*omega2 - mp*G*(rP1*cos(theta) + rP3*sin(theta)) + mDL - rP3*u1 + rP1*u3);
    dv1 = 1 / m1 * (-m3*v3*omega2 - PP3*omega2 - m0*G*sin(theta) + lift * sin(alpha) - drag * cos(alpha) - u1);
    dv3 = 1 / m3 * (m1*v1*omega2 - PP1*omega2 + m0*G*cos(theta) - lift * cos(alpha) - drag * sin(alpha) - u3);
    drP1 = 1 / mp * PP1 - v1 - rP3*omega2;
    drP2 = 1 / mp * PP3 - v3 + rP1*omega2;
    dPP1 = u1;
    dPP3 = u3;
    dmb = u4;
}
