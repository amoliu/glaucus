#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

void rate(double *res, double t, double *y, int l, void *arg)
{
    (void)arg;
    res[0] = sin(t) + cos(y[0]) + sin(y[1]);
    res[1] = cos(t) + sin(y[1]);
}

int main(void)
{
	double *y, *yy, t, y2;
	double t0 = 0, t1 = 20, dt = .2;
	int i, n = 1 + (t1 - t0)/dt;
	y = malloc(sizeof(double) * n * 2);
	yy = malloc(sizeof(double) * n * 2);
 
	for (y[0] = -1, y[1] = 1, i = 1; i < n; i++)
		odesolve(rate, y + 2 * i, dt, t0 + dt * (i - 1), (y + (2 * i - 2)), 2, NULL);
 
	printf("t\ty1\ty2\n");
	for (i = 0; i < n; i += 1) {
		t = t0 + dt * (i);
		printf("%g\t%g\t%g\n", t, y[2*i], y[2*i+1]);
	}
 
	return 0;
}
