/*
Класс точки (вектора) в 3д пространстве
*/
#pragma once

#include <cmath>
#include <iostream>

using namespace std;

class Point
{
public:
    double x, y, z;
    // конструктор
    Point(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    // квадрат модуля
    double mag2() const
    {
        return x * x + y * y + z * z;
    }
    // модуль
    double mag() const
    {
        return sqrt(mag2());
    }
    // угол к оси абсцисс 2х мерного вектора
    double arg() const
    {
        return atan2((*this).y, (*this).x);
    }
    // положительный (чтобы ошибок не было)
    Point operator+() const
    {
        return *this;
    }
    // отрицательный
    Point operator-() const
    {
        return Point(-x, -y, -z);
    }
    // сложить с другим вектором
    Point operator+(const Point &p) const
    {
        return Point(x + p.x, y + p.y, z + p.z);
    }
    // отнять другой вектор
    Point operator-(const Point &p) const
    {
        return Point(x - p.x, y - p.y, z - p.z);
    }
    // умножить на скаляр
    Point operator*(const double a) const
    {
        return Point(x * a, y * a, z * a);
    }
    // поделить на скаляр (если a = 0 сам виноват, фикси основной код)
    Point operator/(const double a) const
    {
        return (*this) * (1 / a);
    }
    // умножить векторно
    Point operator*(const Point &p) const
    {
        return Point(y * p.z - z * p.y, z * p.x - x * p.z, x * p.y - y * p.x);
    }
    // умножить скалярно
    double operator^(const Point &p) const
    {
        return x * p.x + y * p.y + z * p.z;
    }
    // возвести модуль в какую то степень (если 2 то понятно что лучше mag2)
    double operator^(double a) const
    {
        return pow((*this).mag(), a);
    }
    // прибавить угол (повернуть вектор)
    Point operator+(double angle) const
    {
        double len = (*this).mag(), angle0 = (*this).arg();
        return Point(len * cos(angle0 + angle), len * sin(angle0 + angle));
    }
    // отнять угол (повернуть вектор)
    Point operator-(double angle) const
    {
        return (*this) + (-angle);
    }
    // сложить с другим вектором
    Point &operator+=(const Point &p)
    {
        *this = *this + p;
        return *this;
    }
    // отнять другой вектор
    Point &operator-=(const Point &p)
    {
        *this = *this - p;
        return *this;
    }
    // умножить на скаляр
    Point &operator*=(const double a)
    {
        *this = *this * a;
        return *this;
    }
    // поделить на скаляр (если a = 0 сам виноват, фикси основной код)
    Point &operator/=(const double a)
    {
        *this = *this / a;
        return *this;
    }
    // умножить векторно
    Point &operator*=(const Point &p)
    {
        *this = *this * p;
        return *this;
    }
    // прибавить угол (повернуть вектор)
    Point &operator+=(const double angle)
    {
        *this = *this + angle;
        return *this;
    }
    // отнять угол (повернуть вектор)
    Point &operator-=(const double angle)
    {
        *this = *this - angle;
        return *this;
    }
    // единичный вектор (опять же если 0 то сам виноват и иди код фиксить)
    Point unity() const
    {
        return (*this) / (*this).mag();
    }
    // вывести на экран
    friend ostream& operator<<(ostream& os, const Point& p) {
        os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
        return os;
    }
};