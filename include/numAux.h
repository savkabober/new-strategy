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
    inline double sgn(double a) {
        if (a > 0) return 1;
        if (a < 0) return -1;
        return 0;
    }
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
        prod[0] = -a / b;
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
    }
    //решить уравнение 3 степени
    int solveThree(double *prod, double a, double b, double c, double d) {
        if (a == 0) {
            return solveTwo(prod, b, c, d);
        }
        double Q = (a * a - 3 * b) / 9;
        double R = (2 * a * a * a - 9 * a * b + 27 * c) / 54;
        double S = Q * Q * Q - R * R;
        if (S > 0) {
            double phi = acos(R / sqrt(Q * Q * Q)) / 3;
            prod[0] = -2 * sqrt(Q) * cos(phi - 2 * M_PI / 3) - a / 3;
            prod[1] = -2 * sqrt(Q) * cos(phi) - a / 3;
            prod[2] = -2 * sqrt(Q) * cos(phi + 2 * M_PI / 3) - a / 3;
            quicksort::quicksort(prod, 0, 3);
            return 3;
        }
        if (S < 0) {
            if (Q > 0) {
                prod[0] = -2 * sgn(R) * sqrt(Q) * cosh(acosh(abs(R) / sqrt(Q * Q * Q)) / 3) - a / 3;
                return 1;
            }
            if (Q < 0) {
                prod[0] = -2 * sgn(R) * sqrt(-Q) * sinh(asinh(abs(R) / sqrt(-Q * Q * Q)) / 3) - a / 3;
                return 1;
            }
            prod[0] = -cbrt(c - a * a * a / 27) - a / 3;
            return 1;
        }
        prod[0] = -2 * cbrt(R) - a / 3;
        prod[1] = cbrt(R) - a / 3;
        quicksort::quicksort(prod, 0, 2);
        return 2;
    }
    //решить уравнение 4 степени
    // int solveFour(double *prod, double a, double b, double c, double d, double e) {

    // }
}
