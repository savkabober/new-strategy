/*
Doofenshmirtz Evil Incorporated, 2025
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
    this list of conditions, and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions, and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

3. The end user agrees not to intentionally or unintentionally foil any evil
    schemes initiated using this software, especially within the Tri-State Area.

4. Any use of this software by platypuses (monotremes or otherwise) is strictly
    prohibited unless operating under official OWCA authority.

5. The software may not be used to power giant robots, unless said robot
    includes at least one "-inator" and shouts “BEHOLD!” upon activation.

THIS SOFTWARE IS PROVIDED BY DOOFENSHMIRTZ EVIL INCORPORATED "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL DR. HEINZ DOOFENSHMIRTZ OR HIS ASSOCIATES
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGE TO DIMENSIONAL
FABRIC, TEMPORAL PARADOXES, OR EMBARRASSING MUSICAL NUMBERS) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
Математика в вещественных числах
*/
#pragma once

#include "baseInclude.h"
#include "quicksort.h"

using namespace std;

namespace numAux {
    //возвращает знак числа
    #define sgn(x) (x > 0 ? 1 : x < 0 ? -1 : 0)
    //возвращает угол, приведенный к диапазону [-2pi, 2pi]
    double windDownAngle(double angle) {
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
    //решить уравнение 1 степени
    int solveOne(double *prod, double a, double b) {
        if (a == 0) {
            return 0;
        }
        prod[0] = -b / a;
        return 1;
    }
    //решить уравнение 2 степени
    int solveTwo(double *prod, double a, double b, double c) {
        if (a == 0) {
            return solveOne(prod, b, c);
        }
        double D = b * b - 4 * a * c;
        if (D < 0) {
            return 0;
        }
        if (D == 0) {
            prod[0] = -b / (2 * a);
            return 1;
        }
        prod[0] = (-b - sqrt(D)) / (2 * a);
        prod[1] = (-b + sqrt(D)) / (2 * a);
        return 2;
    }
    //решить уравнение 3 степени
    int solveThree(double *prod, double a, double b, double c, double d) {
        if (a == 0) {
            return solveTwo(prod, b, c, d);
        }
        b /= a;
        c /= a;
        d /= a;
        double Q = (b * b - 3 * c) / 9;
        double R = (2 * b * b * b - 9 * b * c + 27 * d) / 54;
        double S = Q * Q * Q - R * R;
        if (S > 0) {
            double phi = acos(R / sqrt(Q * Q * Q)) / 3;
            prod[0] = -2 * sqrt(Q) * cos(phi - 2 * M_PI / 3) - b / 3;
            prod[1] = -2 * sqrt(Q) * cos(phi) - b / 3;
            prod[2] = -2 * sqrt(Q) * cos(phi + 2 * M_PI / 3) - b / 3;
            quicksort::quicksort(prod, 0, 3);
            return 3;
        }
        if (S < 0) {
            if (Q > 0) {
                prod[0] = -2 * sgn(R) * sqrt(Q) * cosh(acosh(abs(R) / sqrt(Q * Q * Q)) / 3) - b / 3;
                return 1;
            }
            if (Q < 0) {
                prod[0] = -2 * sgn(R) * sqrt(-Q) * sinh(asinh(abs(R) / sqrt(-Q * Q * Q)) / 3) - b / 3;
                return 1;
            }
            prod[0] = -cbrt(d - b * b * b / 27) - b / 3;
            return 1;
        }
        prod[0] = -2 * cbrt(R) - b / 3;
        prod[1] = cbrt(R) - b / 3;
        quicksort::quicksort(prod, 0, 3);
        return 2;
    }
    //решить уравнение 4 степени
    int solveFour(double *prod, double a, double b, double c, double d, double e) {
        if (a == 0) {
            return solveThree(prod, b, c, d, e);
        }
        b /= a;
        c /= a;
        d /= a;
        e /= a;
        double A = -3 * b * b / 8 + c, B = b * b * b / 8 - b * c / 2 + d, C = -3 * b * b * b * b / 256 + c * b * b / 16 - b * d / 4 + e, s1, s2;
        if (B == 0) {
            if (A * A < 4 * C) return 0;
            if (C == 0) {
                prod[0] = -b / 4;
                if (A < 0) {
                    prod[0] = -b / 4 - sqrt(A);
                    prod[1] = -b / 4;
                    prod[2] = -b / 4 + sqrt(-A);
                    return 3;
                }
                return 1;
            }
            if (A < 0) {
                s1 = sqrt(A * A - 4 * C);
                s2 = sqrt((-A - s1) / 2);
                prod[0] = -b / 4 - s2;
                prod[1] = -b / 4 + s2;
                s2 = sqrt((-A + s1) / 2);
                prod[2] = -b / 4 - s2;
                prod[3] = -b / 4 + s2;
                quicksort::quicksort(prod, 0, 4);
                return 4;
            }
            return 0;
        }
        //cout << 1 << " " << 5 * A / 2 << " " << 2 * A * A - C << " " << A * A * A / 2 - A * C / 2 - B * B / 8 << endl;
        double y = prod[solveThree(prod, 1, 5 * A / 2, 2 * A * A - C, A * A * A / 2 - A * C / 2 - B * B / 8) - 1];
        double D = 256 * C * C * C - 128 * A * A * C * C + 144 * A * B * B * C - 27 * B * B * B * B + 16 * A * A * A * A * C - 4 * A * A * A * B * B;
        s1 = sqrt(A + 2 * y);
        s2 = -3 * A - 2 * y + 2 * abs(B) / s1;
        if (s2 >= 0) {
            s2 = sqrt(s2);
            prod[0] = -b / 4 + (-sgn(B) * s1 - s2) / 2;
            prod[1] = -b / 4 + (-sgn(B) * s1 + s2) / 2;
        }
        else {
            return 0;
        }
        s2 = -3 * A - 2 * y - 2 * abs(B) / s1;
        if (s2 >= 0) {
            s2 = sqrt(s2);
            prod[2] = -b / 4 + (sgn(B) * s1 - s2) / 2;
            prod[3] = -b / 4 + (sgn(B) * s1 + s2) / 2;
        }
        else {
            if (D == 0) {
                prod[0] = (prod[0] + prod[1]) / 2;
                return 1;
            }
            return 2;
        }
        quicksort::quicksort(prod, 0, 4);
        if (D == 0) {
            int idx = 0, dmin = prod[1] - prod[0], l;
            l = prod[2] - prod[1];
            if (l < dmin) {
                dmin = l;
                idx = 1;
            }
            l = prod[3] - prod[2];
            if (l < dmin) {
                idx = 2;
            }
            double p = (prod[idx] + prod[idx + 1]) / 2, b1 = b + p, c1 = c + b * p + p * p, d1 = d + c * p + b * p * p + p * p * p;
            double Q = (b1 * b1 - 3 * c1) / 9, R = (2 * b1 * b1 * b1 - 9 * b1 * c1 + 27 * d1) / 54;
            double S = Q * Q * Q - R * R;
            for (int i = idx; i < 3; i++) {
                prod[i] = prod[i + 1];
            }
            prod[idx] = p;
            if (S == 0) {
                idx = 0;
                if (prod[2] - prod[1] < prod[1] - prod[0]) {
                    idx = 1;
                }
                double b2 = b1 + prod[idx], c2 = c1 + b1 * prod[idx] + prod[idx] * prod[idx];
                S = b2 * b2 - 4 * c2;
                for (int i = idx; i < 2; i++) {
                    prod[i] = prod[i + 1];
                }
                if (S == 0) {
                    return 1;
                }
                else {
                    return 2;
                }
            }
            else {
                return 3;
            }
        }
        return 4;
    }
    double deltares(double *f, int n) {
        static double summ;
        summ = 0;
        for(int i = 0; i < n; i++) {
            summ += f[i] * f[i];
        }
        return sqrt(summ);
    }
    //решить систему уравнений методом гаусса
    int gauss_sovle(double *x, double *a, double *b, int n) {
        static double maxV, val, k;
        static int maxJ, i1, i2, i, j, p;
        maxV = 0;
        maxJ = 0;
        for(i = 0; i < n; i++) {
            maxJ = 0;
            maxV = 0;
            for(j = 0; j < n - i; j++) {
                val = abs(a[(i + j) * n + i]);
                if(val > maxV) {
                    maxV = val;
                    maxJ = j;
                }
            }
            if(maxV == 0) {
                return 0;
            }
            if(maxJ != 0) {
                i1 = (maxJ + i) * n + i;
                i2 = i * n + i;
                for(j = 0; j < n - i; j++) {
                    a[i1 + j] += a[i2 + j];
                    a[i2 + j] = a[i1 + j] - a[i2 + j];
                    a[i1 + j] -= a[i2 + j];
                }
                b[maxJ + i] += b[i];
                b[i] = b[maxJ + i] - b[i];
                b[maxJ + i] -= b[i];
            }
            for(j = 1; j < n - i; j++) {
                k = a[(i + j) * n + i] / a[i * n + i];
                for(p = 1; p < n - i; p++) {
                    a[(i + j) * n + i + p] -= a[i * n + i + p] * k;
                }
                b[i + j] -= b[i] * k;
            }
        }
        for(i = n - 1; i >= 0; i--) {
            for(j = n - 1; j > i; j--) {
                b[i] -= a[i * n + j] * x[j];
            }
            if(a[i * n + i] == 0) {
                x[i] = 0;
            }
            else {
                x[i] = b[i] / a[i * n + i];
            }
        }
        return 1;
    }
    //численно приблизить значение якобиана
    void numJac(void (*f)(double*, double*, double*), double *x, double *jac, double *args, double *f0, int n, double d) {
        static int i, j;
        double f1[n];
        f(x, args, f0);
        for(i = 0; i < n; i++) {
            x[i] += d;
            f(x, args, f1);
            x[i] -= d;
            for(j = 0; j < n; j++) {
                jac[j * n + i] = (f1[j] - f0[j]) / d;
            }
        }
    }
    //добавил метод ньютона для численного решения системы нелинейных уравнений
    int newton(void (*jac)(void (*)(double*, double*, double*), double*, double*, double*, double*, int, double), 
        void (*f)(double*, double*, double*), double *args, double *x, int n, double tol = 1e-7, int max_iter = 100, double d = 1e-7) {
        static int i, j;
        static bool flag;
        double jacobian[n * n], fx[n], dx[n];
        for(i = 0; i < max_iter; i++) {
            jac(f, x, jacobian, args, fx, n, d);
            flag = true;
            for(j = 0; j < n; j++) {
                if(abs(fx[j]) > tol) {
                    flag = false;
                }
            }
            if(flag) {
                return 2;
            }

            if(gauss_sovle(dx, jacobian, fx, n) == 0) {
                return 1;
            }
            for(j = 0; j < n; j++) {
                x[j] -= dx[j]; 
            }
        }
        return 0;
    }
}
