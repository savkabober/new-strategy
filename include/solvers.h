/*
Численные решалки нелинейных уравнений
*/
#pragma once

#include "auxiliary/numAux.h"

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
    void jac1(void (*f)(double *, double *, double *), double *Vm, double *jac, double *args, double *fx, int n, double d)
    {
        static double ma, mb;
        ma = sqrt((args[0] - Vm[0]) * (args[0] - Vm[0]) + (args[1] - Vm[1]) * (args[1] - Vm[1]));
        mb = sqrt((args[2] - Vm[0]) * (args[2] - Vm[0]) + (args[3] - Vm[1]) * (args[3] - Vm[1]));
        fx[0] = 2 * args[6] * args[4] - (Vm[0] + args[0]) * ma - (Vm[0] + args[2]) * mb;
        fx[1] = 2 * args[6] * args[5] - (Vm[1] + args[1]) * ma - (Vm[1] + args[3]) * mb;
        jac[0] = -ma - (Vm[0] * Vm[0] - args[0] * args[0]) / ma - mb - (Vm[0] * Vm[0] - args[2] * args[2]) / mb;
        jac[1] = -(Vm[0] + args[0]) * (Vm[1] - args[1]) / ma - (Vm[0] + args[2]) * (Vm[1] - args[3]) / mb;
        jac[2] = -(Vm[1] + args[1]) * (Vm[0] - args[0]) / ma - (Vm[1] + args[3]) * (Vm[0] - args[2]) / mb;
        jac[3] = -ma - (Vm[1] * Vm[1] - args[1] * args[1]) / ma - mb - (Vm[1] * Vm[1] - args[3] * args[3]) / mb;
    }

    void jac2(void (*f)(double *, double *, double *), double *ang, double *jac, double *args, double *fx, int n, double d)
    {
        static double ma, mb, dma, dmb, lhs[2], dl[2], Vm[2], ln, c2, s2; //, tc2, ts2, tVm[2], tma, tmb, tlhs[2];
        c2 = cos(ang[0]);
        s2 = sin(ang[0]);
        Vm[0] = c2 * args[7];
        Vm[1] = s2 * args[7];
        ma = sqrt((args[0] - Vm[0]) * (args[0] - Vm[0]) + (args[1] - Vm[1]) * (args[1] - Vm[1]));
        mb = sqrt((args[2] - Vm[0]) * (args[2] - Vm[0]) + (args[3] - Vm[1]) * (args[3] - Vm[1]));
        lhs[0] = 2 * args[6] * args[4] - (Vm[0] + args[0]) * ma - (Vm[0] + args[2]) * mb;
        lhs[1] = 2 * args[6] * args[5] - (Vm[1] + args[1]) * ma - (Vm[1] + args[3]) * mb;
        ln = sqrt(lhs[0] * lhs[0] + lhs[1] * lhs[1]);
        fx[0] = (lhs[0] * c2 + lhs[1] * s2) / ln;
        dma = (Vm[1] * args[0] - Vm[0] * args[1]) / ma;
        dmb = (Vm[1] * args[2] - Vm[0] * args[3]) / mb;
        dl[0] = Vm[1] * ma - (Vm[0] + args[0]) * dma + Vm[1] * mb - (Vm[0] + args[2]) * dmb;
        dl[1] = -Vm[0] * ma - (Vm[1] + args[1]) * dma - Vm[0] * mb - (Vm[1] + args[3]) * dmb;
        jac[0] = -((-s2 * lhs[0] + c2 * dl[0] + c2 * lhs[1] + s2 * dl[1]) / ln - (lhs[0] * dl[0] + lhs[1] * dl[1]) * fx[0] / (ln * ln)); // / sqrt(1 - (fx[0]  * fx[0]));
        fx[0] = -fx[0] + 1;
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

            if (numAux::gaussSovle(dx, jacobian, fx, n) == 0)
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