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
    int sgn(double value) {
        return value > 0 ? 1 : value < 0 ? -1 : 0;
    }
    int solveOne(double *prod, double a, double b);
    int solveTwo(double *prod, double a, double b, double c);
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
    complex<double> evalPolyComplex(const vector<double> &coefficients, complex<double> value) {
        complex<double> result = 0.0;
        for (double coefficient : coefficients) {
            result = result * value + coefficient;
        }
        return result;
    }
    double evalPolyReal(const vector<double> &coefficients, double value) {
        double result = 0.0;
        for (double coefficient : coefficients) {
            result = result * value + coefficient;
        }
        return result;
    }
    int solvePolynomialReal(double *prod, vector<double> coefficients) {
        static constexpr double root_eps = 1e-7;
        while (coefficients.size() > 1 && abs(coefficients.front()) < root_eps) {
            coefficients.erase(coefficients.begin());
        }
        const int degree = static_cast<int>(coefficients.size()) - 1;
        if (degree <= 0) {
            return 0;
        }
        if (degree == 1) {
            return solveOne(prod, coefficients[0], coefficients[1]);
        }
        if (degree == 2) {
            return solveTwo(prod, coefficients[0], coefficients[1], coefficients[2]);
        }

        const double leading = coefficients.front();
        for (double &coefficient : coefficients) {
            coefficient /= leading;
        }

        vector<complex<double>> roots(degree);
        const complex<double> seed(0.4, 0.9);
        for (int i = 0; i < degree; ++i) {
            roots[i] = pow(seed, i + 1);
        }

        for (int iteration = 0; iteration < 80; ++iteration) {
            bool converged = true;
            for (int i = 0; i < degree; ++i) {
                complex<double> denominator = 1.0;
                for (int j = 0; j < degree; ++j) {
                    if (i == j) {
                        continue;
                    }
                    denominator *= (roots[i] - roots[j]);
                }
                if (abs(denominator) < 1e-12) {
                    denominator = complex<double>(1e-12, 1e-12);
                }
                const complex<double> delta = evalPolyComplex(coefficients, roots[i]) / denominator;
                roots[i] -= delta;
                if (abs(delta) > 1e-12) {
                    converged = false;
                }
            }
            if (converged) {
                break;
            }
        }

        vector<double> realRoots;
        for (const complex<double> &root : roots) {
            if (abs(root.imag()) > 1e-6) {
                continue;
            }

            double real = root.real();
            for (int polish = 0; polish < 3; ++polish) {
                double derivative = 0.0;
                double value = 0.0;
                for (size_t i = 0; i < coefficients.size(); ++i) {
                    value = value * real + coefficients[i];
                    if (i + 1 < coefficients.size()) {
                        derivative = derivative * real + coefficients[i] * static_cast<double>(coefficients.size() - i - 1);
                    }
                }
                if (abs(derivative) < 1e-12) {
                    break;
                }
                real -= value / derivative;
            }

            if (abs(evalPolyReal(coefficients, real)) < 1e-4) {
                realRoots.push_back(real);
            }
        }

        sort(realRoots.begin(), realRoots.end());
        vector<double> uniqueRoots;
        for (double root : realRoots) {
            if (uniqueRoots.empty() || abs(root - uniqueRoots.back()) > 1e-5) {
                uniqueRoots.push_back(root);
            }
        }

        for (size_t i = 0; i < uniqueRoots.size(); ++i) {
            prod[i] = uniqueRoots[i];
        }
        return static_cast<int>(uniqueRoots.size());
    }
    //решить уравнение 1 степени
    int solveOne(double *prod, double a, double b) {
        if (abs(a) < 1e-12) {
            return 0;
        }
        prod[0] = -b / a;
        return 1;
    }
    //решить уравнение 2 степени
    int solveTwo(double *prod, double a, double b, double c) {
        if (abs(a) < 1e-12) {
            return solveOne(prod, b, c);
        }
        double D = b * b - 4 * a * c;
        if (D < -1e-12) {
            return 0;
        }
        if (abs(D) < 1e-12) {
            prod[0] = -b / (2 * a);
            return 1;
        }
        const double sqrtD = sqrt(max(0.0, D));
        prod[0] = (-b - sqrtD) / (2 * a);
        prod[1] = (-b + sqrtD) / (2 * a);
        return 2;
    }
    //решить уравнение 3 степени
    int solveThree(double *prod, double a, double b, double c, double d) {
        return solvePolynomialReal(prod, {a, b, c, d});
    }
    //решить уравнение 4 степени
    int solveFour(double *prod, double a, double b, double c, double d, double e) {
        return solvePolynomialReal(prod, {a, b, c, d, e});
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
        vector<double> f1(n, 0.0);
        f(x, args, f0);
        for(i = 0; i < n; i++) {
            x[i] += d;
            f(x, args, f1.data());
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
        vector<double> jacobian(n * n, 0.0), fx(n, 0.0), dx(n, 0.0);
        for(i = 0; i < max_iter; i++) {
            jac(f, x, jacobian.data(), args, fx.data(), n, d);
            flag = true;
            for(j = 0; j < n; j++) {
                if(abs(fx[j]) > tol) {
                    flag = false;
                }
            }
            if(flag) {
                return 2;
            }

            if(gauss_sovle(dx.data(), jacobian.data(), fx.data(), n) == 0) {
                return 1;
            }
            for(j = 0; j < n; j++) {
                x[j] -= dx[j];
            }
        }
        return 0;
    }
}
