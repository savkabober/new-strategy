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
Математика в векторах
*/
#pragma once

#include "baseInclude.h"
#include "Point.h"
#include "numAux.h"

using namespace std;

namespace auxiliary {
    //возвращает ближайшую точку к прямой. Типы: 'S' - сегмент, 'R' - луч (из 1 точки во вторую), 'L' - прямая
    Point closestPointOnLine(const Point &p1, const Point &p2, const Point &p, char type = 'S') {
        double prj = (p2 - p1)^(p-p1);
        if (prj < 0 && type != 'L') {
            return p1;
        }
        double len = (p2 - p1).mag();
        if (prj > len && type == 'S') {
            return p2;
        }
        return (p1-p2) / len * prj + p1;
    }
    //возвращает пересечение 2 прямых если оно существует. Типы: 'S' - сегмент, 'R' - луч (из 1 точки во вторую), 'L' - прямая
    bool getLineIntersection(Point &prod, const Point &p1, const Point &p2, const Point &p3, const Point &p4, char type1 = 'S', char type2 = 'S') {
        Point d1 = p2 - p1, d2 = p4 - p3, d = p3 - p1;
        double determinant = (d2 * d1).z;
        if (determinant == 0) {
            return false;
        }
        double t1 = (d2 * d).z / determinant;
        double t2 = (d1 * d).z / determinant;
        bool valid = true;
        if (t1 < 0 && type1 != 'L' || t1 > 1 && type1 == 'S') {
            valid = false;
        }
        if (t2 < 0 && type2 != 'L' || t2 > 1 && type2 == 'S') {
            valid = false;
        }
        if (valid) {
            prod = p1 + (p2 - p1) * t1;
            return true;
        }
        return false;
    }
    //возвращает нормированный угол между прямыми
    double getAngleBetweenPoints(Point a, Point b, Point c)
    {
        return numAux::windDownAngle((a - b).arg() - (c - b).arg());
    }
    //возвращает пересечения двух окружностей
    int circlesIntersection(Point *prod, Point p1, Point p2, double r1, double r2) {
        double d = (p1 - p2).mag();
        if (d > r1 + r2) {
            return 0;
        }
        if (d == r1 + r2) {
            prod[0] = p1 + (p2 - p1) * r1;
            return 1;
        }
        double a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
        double h = sqrtf(r1 * r1 - a * a);
        double x = p1.x + a * (p2.x - p1.x) / d;
        double y = p1.y + a * (p2.y - p1.y) / d;
        prod[0].x = x + h * (p2.y - p1.y) / d;
        prod[0].y = y - h * (p2.x - p1.x) / d;
        prod[1].x = x - h * (p2.y - p1.y) / d;
        prod[1].y = y + h * (p2.x - p1.x) / d;
        return 2;
    }
    //возвращает касательные через точку p к окружности c радиуса r 
    int getTangentPoints(Point *prod, Point p, Point c, double r)
    {
        return circlesIntersection(prod, c, (p + c) / 2, r, (p - c).mag() / 2);;
    }
    //возвращает ближайшую к многоугольнику точку
    Point nearestPointOnPoly(Point p, Point *poly, int n)
    {
        double minD, d;
        Point ans, pnt;
        ans = closestPointOnLine(poly[0], poly[n - 1], p);
        minD = (ans - p).mag();
        for (int i = 1; i < n; i++)
        {
            pnt = closestPointOnLine(poly[i], poly[i - 1], p);
            d = (pnt - p).mag();
            if (d < minD)
            {
                minD = d;
                ans = pnt;
            }
        }
        return ans;
    }
    //возвращает, находится ли точка в выпуклом многоугольнике
    bool isPointInsidePoly(Point p, Point *poly, int n)
    {
        double old_sign = numAux::sgn(((p - poly[n - 1]) * (poly[0] - poly[n - 1])).z);
        for (int i = 1; i < n; i++)
        {
            if (old_sign != numAux::sgn(((p - poly[i - 1]) * (poly[i] - poly[i - 1])).z))
            {
                return false;
            }
        }
        return true;
    }
}