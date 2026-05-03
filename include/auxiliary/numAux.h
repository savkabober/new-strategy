/*
Математика в вещественных числах
*/
#pragma once

#include <cmath>
#include "sort.h"

using namespace std;

namespace numAux
{
    // в квадрат возвести
    double square(double x)
    {
        return x * x;
    }
    // возвращает знак числа
    int sgn(double x)
    {
        if (x > 0)
            return 1;
        if (x < 0)
            return -1;
        return 0;
    }
    // возвращает угол, приведенный к диапазону [-2pi, 2pi]
    double windDownAngle(double angle)
    {
        if (abs(angle) > 2 * M_PI)
        {
            angle = fmod(angle, 2 * M_PI);
        }
        if (abs(angle) > M_PI)
        {
            angle -= 2 * M_PI * sgn(angle);
        }
        return angle;
    }
    // решить уравнение 1 степени
    int solveEq(double *prod, double a, double b)
    {
        if (a == 0)
        {
            return 0;
        }
        prod[0] = -b / a;
        return 1;
    }
    // решить уравнение 2 степени
    int solveEq(double *prod, double a, double b, double c)
    {
        if (a == 0)
        {
            return solveEq(prod, b, c);
        }
        double D = b * b - 4 * a * c;
        if (D < 0)
        {
            return 0;
        }
        prod[0] = (-b - sqrt(D)) / (2 * a);
        prod[1] = (-b + sqrt(D)) / (2 * a);
        return 2;
    }
    // решить уравнение 3 степени
    int solveEq(double *prod, double a, double b, double c, double d)
    {
        if (a == 0)
        {
            return solveEq(prod, b, c, d);
        }
        b /= a;
        c /= a;
        d /= a;
        double Q = (b * b - 3 * c) / 9;
        double R = (2 * b * b * b - 9 * b * c + 27 * d) / 54;
        double S = Q * Q * Q - R * R;
        double sqrtQ = sqrt(abs(Q));
        if (S >= 0)
        {
            double phi = acos(R / (sqrtQ * sqrtQ * sqrtQ)) / 3;
            prod[0] = -2 * sqrtQ * cos(phi - 2 * M_PI / 3) - b / 3;
            prod[1] = -2 * sqrtQ * cos(phi) - b / 3;
            prod[2] = -2 * sqrtQ * cos(phi + 2 * M_PI / 3) - b / 3;
            sort::sort3(prod);
            return 3;
        }
        else
        {
            if (Q > 0)
            {
                prod[0] = -2 * sgn(R) * sqrtQ * cosh(acosh(abs(R) / (sqrtQ * sqrtQ * sqrtQ)) / 3) - b / 3;
                return 1;
            }
            if (Q < 0)
            {
                prod[0] = -2 * sgn(R) * sqrtQ * sinh(asinh(abs(R) / (sqrtQ * sqrtQ * sqrtQ)) / 3) - b / 3;
                return 1;
            }
            prod[0] = -cbrt(d - b * b * b / 27) - b / 3;
            return 1;
        }
    }
    // решить уравнение 4 степени
    int solveEq(double *prod, double a, double b, double c, double d, double e)
    {
        if (a == 0)
        {
            return solveEq(prod, b, c, d, e);
        }
        b /= a;
        c /= a;
        d /= a;
        e /= a;
        double A = -3 * b * b / 8 + c, B = b * b * b / 8 - b * c / 2 + d, C = -3 * b * b * b * b / 256 + c * b * b / 16 - b * d / 4 + e, s1, s2;
        if (B == 0)
        {
            if (A * A < 4 * C)
                return 0;
            s1 = sqrt(A * A - 4 * C);
            if (A <= 0)
            {
                s2 = sqrt((-A - s1) / 2);
                prod[0] = -b / 4 - s2;
                prod[1] = -b / 4 + s2;
                s2 = sqrt((-A + s1) / 2);
                prod[2] = -b / 4 - s2;
                prod[3] = -b / 4 + s2;
                sort::sort4(prod);
                return 4;
            }
            else if (c <= 0)
            {
                s2 = sqrt((-A + s1) / 2);
                prod[0] = -b / 4 - s2;
                prod[1] = -b / 4 + s2;
                return 2;
            }
            return 0;
        }
        double y = prod[solveEq(prod, 1, 5 * A / 2, 2 * A * A - C, A * A * A / 2 - A * C / 2 - B * B / 8) - 1];
        s1 = sqrt(A + 2 * y);
        s2 = -3 * A - 2 * y + 2 * abs(B) / s1;
        if (s2 >= 0)
        {
            s2 = sqrt(s2);
            prod[0] = -b / 4 + (-sgn(B) * s1 - s2) / 2;
            prod[1] = -b / 4 + (-sgn(B) * s1 + s2) / 2;
        }
        else
        {
            return 0;
        }
        s2 = -3 * A - 2 * y - 2 * abs(B) / s1;
        if (s2 >= 0)
        {
            s2 = sqrt(s2);
            prod[2] = -b / 4 + (sgn(B) * s1 - s2) / 2;
            prod[3] = -b / 4 + (sgn(B) * s1 + s2) / 2;
        }
        else
        {
            return 2;
        }
        sort::sort3(prod);
        return 4;
    }
    double deltares(double *f, int n)
    {
        double summ;
        summ = 0;
        for (int i = 0; i < n; i++)
        {
            summ += f[i] * f[i];
        }
        return sqrt(summ);
    }
    // решить систему уравнений методом гаусса
    int gaussSovle(double *x, double *a, double *b, int n)
    {
        double maxV, val, k;
        int maxJ, i1, i2, i, j, p;
        maxV = 0;
        maxJ = 0;
        for (i = 0; i < n; i++)
        {
            maxJ = 0;
            maxV = 0;
            for (j = 0; j < n - i; j++)
            {
                val = abs(a[(i + j) * n + i]);
                if (val > maxV)
                {
                    maxV = val;
                    maxJ = j;
                }
            }
            if (maxV == 0)
            {
                return 0;
            }
            if (maxJ != 0)
            {
                i1 = (maxJ + i) * n + i;
                i2 = i * n + i;
                for (j = 0; j < n - i; j++)
                {
                    a[i1 + j] += a[i2 + j];
                    a[i2 + j] = a[i1 + j] - a[i2 + j];
                    a[i1 + j] -= a[i2 + j];
                }
                b[maxJ + i] += b[i];
                b[i] = b[maxJ + i] - b[i];
                b[maxJ + i] -= b[i];
            }
            for (j = 1; j < n - i; j++)
            {
                k = a[(i + j) * n + i] / a[i * n + i];
                for (p = 1; p < n - i; p++)
                {
                    a[(i + j) * n + i + p] -= a[i * n + i + p] * k;
                }
                b[i + j] -= b[i] * k;
            }
        }
        for (i = n - 1; i >= 0; i--)
        {
            for (j = n - 1; j > i; j--)
            {
                b[i] -= a[i * n + j] * x[j];
            }
            if (a[i * n + i] == 0)
            {
                x[i] = 0;
            }
            else
            {
                x[i] = b[i] / a[i * n + i];
            }
        }
        return 1;
    }
}
