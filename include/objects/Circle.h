/*
Круг
*/
#pragma once

#include "Object.h"

class Circle : public Object {
    protected:
    double rad;
    public:
    Circle(Point p = Point(), double r = 0) : Object(p), rad(r) {}
    //геттеры
    Point get_rad() const {
        return rad;
    }
    //сеттеры
    void set_rad(double r) {
        rad = r;
    }
};