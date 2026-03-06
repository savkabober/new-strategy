/*
Добавил АТТ
*/
#pragma once

#include "Circle.h"

class AbsRigBody : public Circle {
    protected:
    Point vel;
    public:
    AbsRigBody(Point p = Point(), double r = 0, Point v = Point()) : Circle(p, r), vel(v), {}
    //геттеры
    Point get_vel() const {
        return vel;
    }
    //сеттеры
    void set_vel(const Point& v) {
        vel = v;
    }
};