#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#define G 9.80665

inline void vec_add(double *v1, double *v2, int l)
{
    int i;
    for (i = 0; i < l; ++i)
        v1[i] += v2[i];
}

inline void vec_mul(double *v, double a, int l)
{
    int i;
    for (i = 0; i < l; ++i)
        v[i] *= a;
}
 
inline void vec_div(double *v, double a, int l)
{
    int i;
    for (i = 0; i < l; ++i)
        v[i] /= a;
}
 
inline void vec_cpy(double *v1, double *v2, int l)
{
    int i;
    for (i = 0; i < l; ++i)
        v1[i] = v2[i];
}

void odesolve(void(*f)(double *, double, double *, int, void *), double *res, double dt, double t, double *y, int l, void *ctx)
{
    double k[l];
    double yk[l];

    f(res, t, y, l, ctx);
    vec_mul(res, dt, l);
    
    vec_cpy(yk, res, l);
    vec_div(yk, 2, l);
    vec_add(yk, y, l);

    f(k, t + dt / 2, yk, l, ctx);
    vec_mul(k, dt, l);

    vec_cpy(yk, k, l);
    vec_div(yk, 2, l);
    vec_add(yk, y, l);

    vec_mul(k, 2, l);
    vec_add(res, k, l);

    f(k, t + dt / 2, yk, l, ctx);
    vec_mul(k, dt, l);

    vec_cpy(yk, k, l);
    vec_add(yk, y, l);

    vec_mul(k, 2, l);
    vec_add(res, k, l);

    f(k, t + dt, yk, l, ctx);
    vec_mul(k, dt, l);

    vec_add(res, k, l);
    vec_div(res, 6, l);
    vec_add(res, y, l);
}
 
void rate(double *res, double t, double *y, int l, void *arg)
{
    (void)arg;
    res[0] = sin(t) + cos(y[0]) + sin(y[1]);
    res[1] = cos(t) + sin(y[1]);
}

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
 
int main(void)
{
	double *y, *yy, t, y2;
	double t0 = 0, t1 = 20, dt = .2;
	int i, n = 1 + (t1 - t0)/dt;
	y = malloc(sizeof(double) * n * 2);
	yy = malloc(sizeof(double) * n * 2);
 
	for (y[0] = -1, y[1] = 1, i = 1; i < n; i++)
		rk4(rate, y + 2 * i, dt, t0 + dt * (i - 1), (y + (2 * i - 2)), 2, NULL);
 
	printf("t\ty1\ty2\n");
	for (i = 0; i < n; i += 1) {
		t = t0 + dt * (i);
		printf("%g\t%g\t%g\n", t, y[2*i], y[2*i+1]);
	}
 
	return 0;
}
