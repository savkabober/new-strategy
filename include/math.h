#pragma once

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <complex>
#include "const.h"

using namespace std;

#define square(x) ((x) * (x))

struct Point
{   
    public:
    double c[3];
    Point() { c[0] = 0; c[1] = 0; c[2] = 0;}
    Point(double x, double y) {c[0] = x; c[1] = y; c[2] = 0;}
    Point(double x, double y, double z) { c[0] = x; c[1] = y; c[2] = z;}
    Point operator+() const
    {
        return *this;
    }
    Point operator-() const
    {
        return Point(-c[0], -c[1], -c[2]);
    }
    Point operator+(const Point &p) const 
    {
        return Point(c[0] + p.c[0], c[1] + p.c[1], c[2] + p.c[2]);
    }
    Point operator-(const Point &p) const
    {
        return Point(c[0] - p.c[0], c[1] - p.c[1], c[2] - p.c[2]);
    }
    Point operator*(const double a) const
    {
        return Point(c[0] * a, c[1] * a, c[2] * a);
    }
    Point operator/(const double a) const
    {
        return Point(c[0] / a, c[1] / a, c[2] / a);
    }
    double operator*(const Point &p) const
    {
        return c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
    }
    Point &operator+=(const Point &p)
    {
        c[0] += p.c[0];
        c[1] += p.c[1];
        c[2] += p.c[2];
        return *this;
    }
    Point &operator-=(const Point &p)
    {
        c[0] -= p.c[0];
        c[1] -= p.c[1];
        c[2] -= p.c[2];
        return *this;
    }
    Point &operator*=(const double a) {
        c[0] *= a;
        c[1] *= a;
        c[2] *= a;
        return *this;
    }
    static Point vector(const Point &p1, const Point &p2)
    {
        return Point(p1.c[1] * p2.c[2] - p1.c[2] * p2.c[1], p1.c[2] * p2.c[0] - p1.c[0] * p2.c[2], p1.c[0] * p2.c[1] - p1.c[1] * p2.c[0]);
    }
    Point &vectored(const Point &p) {
        *this = vector(*this, p);
        return *this;
    }
    double abs2() const {
        return c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
    }
    double abs() const {
        return sqrt(abs2());
    }
    double operator^(const double a) {
        return pow(abs(), a);
    }
    Point normalized() const {
        double len = abs();
        if (len == 0) {
            return Point();
        }
        return Point(c[0] / len, c[1] / len, c[2] / len);
    }
};

inline Point operator*(double a, const Point p)
{
    return Point(p.c[0] * a, p.c[1] * a, p.c[2] * a);
}

namespace math {
    
}
