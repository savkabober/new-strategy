#pragma once

#include "baseinclude.h"
#include "const.h"

using namespace std;

#define square(x) ((x) * (x))

struct Point
{   
    public:
    double x, y, z;
    Point() : x(0), y(0), z(0) {}
    Point(double x, double y) : x(x), y(y), z(0) {}
    Point(double x, double y, double z) : x(x), y(y), z(z) {}
    double mag2() const {
        return x * x + y * y + z * z;
    }
    double mag() const {
        return sqrt(mag2());
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
        return Point(x / a, y / a, z / a);
    }
    double operator*(const Point &p) const
    {
        return x * x + y * y + z * z;
    }
    double operator^(double a) const {
        return pow((*this).mag(), a);
    }
    Point operator^(const Point &p) const {
        return Point((*this).y * p.z - (*this).z * p.y, (*this).z * p.x - (*this).x * p.z, (*this).x * p.y - (*this).y * p.x);
    }
    Point &operator+=(const Point &p)
    {
        x += p.x;
        y += p.y;
        z += p.z;
        return *this;
    }
    Point &operator-=(const Point &p)
    {
        x -= p.x;
        y -= p.y;
        z -= p.z;
        return *this;
    }
    Point &operator*=(const double a) {
        x *= a;
        y *= a;
        z *= a;
        return *this;
    }
    Point &operator^=(const Point &p) {
        Point a = Point((*this).y * p.z - (*this).z * p.y, (*this).z * p.x - (*this).x * p.z, (*this).x * p.y - (*this).y * p.x);
        *this = a;
        return *this;
    }
    operator double() {
        return (*this).mag();
    }
    Point unity() const {
        double len = (*this).mag();
        if (len == 0) {
            return Point();
        }
        return Point(x / len, y / len, z / len);
    }
};