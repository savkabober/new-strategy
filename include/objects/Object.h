/*
Базовый класс всех объектов
*/
#pragma once

#include "../aux/Point.h"

class Object {
    protected:
    Point pos;
    public:
    Object(Point p = Point) : pos(p) {}
    //геттеры
    Point getPos() const {
        return pos;
    }
    //сеттеры
    void setPos(const Point& p) {
        pos = p;
    }
};