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
    Point getVel() const {
        return vel;
    }
    //сеттеры
    void setVel(const Point& v) {
        vel = v;
    }
};