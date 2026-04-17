/*
Круг
*/
#pragma once

#include "Object.h"

class Circle : public Object
{
protected:
    double rad;

public:
    Circle(Point p = Point(), double r = 0) : Object(p), rad(r) {}
    // геттеры
    double getRad() const
    {
        return rad;
    }
    // сеттеры
    void setRad(double r)
    {
        rad = r;
    }
};