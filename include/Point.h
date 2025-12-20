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
Класс точки (вектора) в 3д пространстве
*/
#pragma once

#include "baseInclude.h"
#include "const.h"

using namespace std;

#define square(x) ((x) * (x))

class Point
{
    public:
    double x, y, z;
    //конструктор
    Point(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    //квадрат модуля
    double mag2() const {
        return x * x + y * y + z * z;
    }
    //модуль
    double mag() const {
        return sqrt(mag2());
    }
    //угол к оси абсцисс 2х мерного вектора
    double arg() const {
        return atan2((*this).y, (*this).x);
    }
    //положительный (чтобы ошибок не было)
    Point operator+() const {
        return *this;
    }
    //отрицательный
    Point operator-() const {
        return Point(-x, -y, -z);
    }
    //сложить с другим вектором
    Point operator+(const Point &p) const  {
        return Point(x + p.x, y + p.y, z + p.z);
    }
    //отнять другой вектор
    Point operator-(const Point &p) const {
        return Point(x - p.x, y - p.y, z - p.z);
    }
    //умножить на скаляр
    Point operator*(const double a) const {
        return Point(x * a, y * a, z * a);
    }
    //поделить на скаляр (если a = 0 сам виноват, фикси основной код)
    Point operator/(const double a) const {
        return (*this) * (1 / a);
    }
    //умножить векторно
    Point operator*(const Point &p) const {
        return Point(y * p.z - z * p.y, z * p.x - x * p.z, x * p.y - y * p.x);
    }
    //умножить скалярно
    double operator^(const Point &p) const {
        return x * p.x + y * p.y + z * p.z;
    }
    //возвести модуль в какую то степень (если 2 то понятно что лучше mag2)
    double operator^(double a) const {
        return pow((*this).mag(), a);
    }
    //прибавить угол (повернуть вектор)
    Point operator+(double angle) const {
        double len = (*this).mag(), angle0 = (*this).arg();
        return Point(len * cos(angle0 + angle), len * sin(angle0 + angle));
    }
    //отнять угол (повернуть вектор)
    Point operator-(double angle) const {
        return (*this) + (-angle);
    }
    //сложить с другим вектором
    Point &operator+=(const Point &p)
    {
        *this = *this + p;
        return *this;
    }
    //отнять другой вектор
    Point &operator-=(const Point &p)
    {
        *this = *this - p;
        return *this;
    }
    //умножить на скаляр
    Point &operator*=(const double a) {
        *this = *this * a;
        return *this;
    }
    //поделить на скаляр (если a = 0 сам виноват, фикси основной код)
    Point &operator/=(const double a) {
        *this = *this / a;
        return *this;
    }
    //умножить векторно
    Point &operator*=(const Point &p) {
        *this = *this * p;
        return *this;
    }
    //прибавить угол (повернуть вектор)
    Point &operator+=(const double angle) {
        *this = *this + angle;
        return *this;
    }
    //отнять угол (повернуть вектор)
    Point &operator-=(const double angle) {
        *this = *this - angle;
        return *this;
    }
    //единичный вектор (опять же если 0 то сам виноват и иди код фиксить)
    Point unity() const {
        return (*this) / (*this).mag();
    }
};