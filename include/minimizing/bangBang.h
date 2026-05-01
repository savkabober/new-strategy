/*
поиск бенгбенга
*/

#pragma once
#include "../auxiliary/numAux.h"
#include "../auxiliary/Point.h"
#include "../solvers.h"

// #include "auxiliary/vecAux.h"
using namespace solvers;
namespace bangBang
{
    void func1(double *Vm, double *args, double *fx)
    {
        double ma, mb;
        ma = sqrt((args[0] - Vm[0]) * (args[0] - Vm[0]) + (args[1] - Vm[1]) * (args[1] - Vm[1]));
        mb = sqrt((args[2] - Vm[0]) * (args[2] - Vm[0]) + (args[3] - Vm[1]) * (args[3] - Vm[1]));
        fx[0] = 2 * args[6] * args[4] - (Vm[0] + args[0]) * ma - (Vm[0] + args[2]) * mb;
        fx[1] = 2 * args[6] * args[5] - (Vm[1] + args[1]) * ma - (Vm[1] + args[3]) * mb;
    }

    void func2(double *ang, double *args, double *fx)
    {
        double Vm[2], lhs[2], ma, mb, ln;
        Vm[0] = cos(ang[0]) * args[7];
        Vm[1] = sin(ang[0]) * args[7];
        ma = sqrt((args[0] - Vm[0]) * (args[0] - Vm[0]) + (args[1] - Vm[1]) * (args[1] - Vm[1]));
        mb = sqrt((args[2] - Vm[0]) * (args[2] - Vm[0]) + (args[3] - Vm[1]) * (args[3] - Vm[1]));
        lhs[0] = 2 * args[6] * args[4] - (Vm[0] + args[0]) * ma - (Vm[0] + args[2]) * mb;
        lhs[1] = 2 * args[6] * args[5] - (Vm[1] + args[1]) * ma - (Vm[1] + args[3]) * mb;
        ln = sqrt(lhs[0] * lhs[0] + lhs[1] * lhs[1]);
        fx[0] = -(lhs[0] * Vm[0] + lhs[1] * Vm[1]) / ln / args[7] + 1;
    }

    Point bangBang(Point start, Point end, Point dr, double Amax, double Vmax, int nshort = 10, int nst = 10, int mult = 2, int barrier = 10000)
    { // gang-bang
        double args[8], angle[1], rn, Vm[2], zero, imin, vmin, vnow[1];
        int i, n;
        args[0] = start.x;
        args[1] = start.y;
        args[2] = end.x;
        args[3] = end.y;
        args[4] = dr.x;
        args[5] = dr.y;
        args[6] = Amax;
        args[7] = Vmax;
        rn = sqrt(args[4] * args[4] + args[5] * args[5]);
        Vm[0] = args[4] / rn * Vmax;
        Vm[1] = args[5] / rn * Vmax;
        Point res = {0, 0};
        int g = 0;
        zero = atan2(args[5], args[4]);
        for (i = 0; g != 2 && i < nshort; i++)
        {
            Vm[0] = cos(zero + 2 * M_PI * i / nshort) * Vmax;
            Vm[1] = sin(zero + 2 * M_PI * i / nshort) * Vmax;
            g = newton(jac1, func1, args, Vm, 2);
        }
        // if (g != 2)
        // {
        //     cout << "g " << g << ", " << start[0] << ", " << start[1] << ", " << end[0] << ", " << end[1] << ", " << args[4] << ", " << args[5] << endl;
        // }
        if (sqrt(Vm[0] * Vm[0] + Vm[1] * Vm[1]) > Vmax * 1.001)
        {
            g = 0;
            zero = atan2(Vm[1], Vm[0]);
            angle[0] = zero;
            g = newton(jac2, func2, args, angle, 1);
            for (n = nst; g != 2 && n <= barrier; n *= mult)
            {
                vmin = 2;
                for (i = 0; i < n; i++)
                {
                    if (n == nst || i % mult != 0)
                    {
                        angle[0] = zero + 2 * M_PI * i / n;
                        func2(angle, args, vnow);
                        if (vnow[0] < vmin)
                        {
                            vmin = vnow[0];
                            imin = i;
                        }
                    }
                }
                angle[0] = zero + 2 * M_PI * imin / n;
                g = newton(jac2, func2, args, angle, 1);
            }
            // if (g != 2)
            // {
            //     cout << "g2 " << g << ", " << start.x << ", " << start.y << ", " << end.x << ", " << end.y << ", " << r.x << ", " << r.y << endl;
            // }

            Vm[0] = cos(angle[0]) * Vmax;
            Vm[1] = sin(angle[0]) * Vmax;
        }
        res.x = Vm[0];
        res.y = Vm[1];
        return res;
    }
}