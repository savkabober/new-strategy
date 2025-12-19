#pragma once

#include "baseinclude.h"
#include "const.h"

using namespace std;

#define square(x) ((x) * (x))

struct Point
{
    public:
    double x, y, z;
    Point(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    double mag2() const {
        return x * x + y * y + z * z;
    }
    double mag() const {
        return sqrt(mag2());
    }
    double arg() const {
        return atan2((*this).y, (*this).x);
    }
    Point operator+() const
    {
        return *this;
    }
    Point operator-() const
    {
        return Point(-x, -y, -z);
    }
    Point operator+(const Point &p) const 
    {
        return Point(x + p.x, y + p.y, z + p.z);
    }
    Point operator-(const Point &p) const
    {
        return Point(x - p.x, y - p.y, z - p.z);
    }
    Point operator*(const double a) const
    {
        return Point(x * a, y * a, z * a);
    }
    Point operator/(const double a) const
    {
        return (*this) * (1 / a);
    }
    Point operator*(const Point &p) const {
        return Point(y * p.z - z * p.y, z * p.x - x * p.z, x * p.y - y * p.x);
    }
    double operator^(const Point &p) const
    {
        return x * p.x + y * p.y + z * p.z;
    }
    double operator^(double a) const {
        return pow((*this).mag(), a);
    }
    Point operator+(double angle) const {
        double len = (*this).mag(), angle0 = (*this).arg();
        return Point(len * cos(angle0 + angle), len * sin(angle0 + angle));
    }
    Point operator-(double angle) const {
        return (*this) + (-angle);
    }
    Point &operator+=(const Point &p)
    {
        *this = *this + p;
        return *this;
    }
    Point &operator-=(const Point &p)
    {
        *this = *this - p;
        return *this;
    }
    Point &operator*=(const double a) {
        *this = *this * a;
        return *this;
    }
    Point &operator/=(const double a) {
        *this = *this / a;
        return *this;
    }
    Point &operator*=(const Point &p) {
        *this = *this * p;
        return *this;
    }
    Point &operator+=(const double angle) {
        *this = *this + angle;
        return *this;
    }
    Point &operator-=(const double angle) {
        *this = *this - angle;
        return *this;
    }
    Point unity() const {
        return (*this) / (*this).mag();
    }
};