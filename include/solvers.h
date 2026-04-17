/*
Численные решалки нелинейных уравнений
*/
#pragma once

#include <numAux.h>

namespace solvers
{
    // численно приблизить значение якобиана
    void numJac(void (*f)(double *, double *, double *), double *x, double *jac, double *args, double *f0, int n, double d)
    {
        int i, j;
        double f1[n];
        f(x, args, f0);
        for (i = 0; i < n; i++)
        {
            x[i] += d;
            f(x, args, f1);
            x[i] -= d;
            for (j = 0; j < n; j++)
            {
                jac[j * n + i] = (f1[j] - f0[j]) / d;
            }
        }
    }
    // добавил метод ньютона для численного решения системы нелинейных уравнений
    int newton(void (*jac)(void (*)(double *, double *, double *), double *, double *, double *, double *, int, double),
               void (*f)(double *, double *, double *), double *args, double *x, int n, double tol = 1e-7, int maxIter = 100, double d = 1e-7)
    {
        int i, j;
        bool flag;
        double jacobian[n * n], fx[n], dx[n];
        for (i = 0; i < maxIter; i++)
        {
            jac(f, x, jacobian, args, fx, n, d);
            flag = true;
            for (j = 0; j < n; j++)
            {
                if (abs(fx[j]) > tol)
                {
                    flag = false;
                }
            }
            if (flag)
            {
                return 2;
            }

            if (gaussSovle(dx, jacobian, fx, n) == 0)
            {
                return 1;
            }
            for (j = 0; j < n; j++)
            {
                x[j] -= dx[j];
            }
        }
        return 0;
    }
}