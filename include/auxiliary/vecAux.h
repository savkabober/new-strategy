/*
Математика в векторах
*/
#pragma once

#include "Point.h"
#include "numAux.h"

using namespace std;

namespace vecAux
{
    // возвращает ближайшую точку к прямой. Типы: 'S' - сегмент, 'R' - луч (из 1 точки во вторую), 'L' - прямая
    Point closestPointOnLine(const Point &p1, const Point &p2, const Point &p, char type = 'S')
    {
        double len = (p2 - p1).mag();
        double prj = (p2 - p1) ^ (p - p1) / len;
        if (prj < 0 && type != 'L')
        {
            return p1;
        }
        if (prj > len && type == 'S')
        {
            return p2;
        }
        return (p2 - p1) / len * prj + p1;
    }
    Point pointOnLine(const Point &p1, const Point &p2, double l)
    {
        return p1 + (p2 - p1).unity() * l;
    }
    // возвращает пересечение 2 прямых если оно существует. Типы: 'S' - сегмент, 'R' - луч (из 1 точки во вторую), 'L' - прямая
    bool getLineIntersection(Point &prod, const Point &p1, const Point &p2, const Point &p3, const Point &p4, char type1 = 'S', char type2 = 'S')
    {
        Point d1 = p2 - p1, d2 = p4 - p3, d = p3 - p1;
        double determinant = (d2 * d1).z;
        if (determinant == 0)
        {
            return false;
        }
        double t1 = (d2 * d).z / determinant;
        double t2 = (d1 * d).z / determinant;
        bool valid = true;
        if ((t1 < 0 && type1 != 'L') || (t1 > 1 && type1 == 'S'))
        {
            valid = false;
        }
        if ((t2 < 0 && type2 != 'L') || (t2 > 1 && type2 == 'S'))
        {
            valid = false;
        }
        if (valid)
        {
            prod = p1 + (p2 - p1) * t1;
            return true;
        }
        return false;
    }
    // возвращает нормированный угол между прямыми
    double getAngleBetweenPoints(const Point &a, const Point &b, const Point &c)
    {
        return numAux::windDownAngle((a - b).arg() - (c - b).arg());
    }
    // возвращает пересечения двух окружностей
    int circlesIntersection(Point *prod, const Point &p1, const Point &p2, double r1, double r2)
    {
        double d = (p1 - p2).mag();
        if (d > r1 + r2)
        {
            return 0;
        }
        if (d == r1 + r2)
        {
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
    // возвращает касательные через точку p к окружности c радиуса r
    int getTangentPoints(Point *prod, const Point &p, const Point &c, double r)
    {
        return circlesIntersection(prod, c, (p + c) / 2, r, (p - c).mag() / 2);
    }
    // возвращает ближайшую к многоугольнику точку
    Point nearestPointOnPoly(const Point &p, const Point *poly, int n)
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
    // возвращает, находится ли точка в выпуклом многоугольнике
    bool isPointInsidePoly(const Point &p, const Point *poly, int n)
    {
        double oldSign = numAux::sgn(((p - poly[n - 1]) * (poly[0] - poly[n - 1])).z);
        for (int i = 1; i < n; i++)
        {
            if (oldSign != numAux::sgn(((p - poly[i - 1]) * (poly[i] - poly[i - 1])).z))
            {
                return false;
            }
        }
        return true;
    }
    // возвращает пересечения прямой и окружности
    int lineCircleIntersect(Point *prod, const Point &p1, const Point &p2, const Point &p, double r, char type = 'S')
    {
        Point h = closestPointOnLine(p1, p2, p, 'L');
        double dist = (h - p).mag();
        if (r < dist)
            return 0;
        if (r == dist)
        {
            prod[0] = h;
            return 1;
        }
        double d = sqrt(r * r - dist * dist);
        Point vec = (p2 - p1).unity() * d;
        int i = 0;
        prod[i] = h + vec;
        if ((((prod[i] - p1) ^ (p2 - p1)) >= 0 || type == 'L') && (((prod[i] - p2) ^ (p1 - p2)) >= 0 || type != 'S'))
            i++;
        prod[i] = h - vec;
        if ((((prod[i] - p1) ^ (p2 - p1)) >= 0 || type == 'L') && (((prod[i] - p2) ^ (p1 - p2)) >= 0 || type != 'S'))
            i++;
        return i;
    }

    // Получить время пересечения параболы и окружности
    inline int parabolaCircleIntersection(double *prod, double rad, const Point &a, const Point &v, const Point &r)
    {
        return numAux::solveEq(prod, a.mag2() / 4, a ^ v, (a ^ r) + v.mag2(), 2 * (v ^ r), r.mag2() - rad * rad);
    }
    inline int lineCircleIntersection(double *prod, double rad, const Point &v, const Point &r)
    {
        return numAux::solveEq(prod, v.mag2(), 2 * (v ^ r), r.mag2() - rad * rad);
    }
}