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
    Point get_pos() const {
        return pos;
    }
    //сеттеры
    void set_pos(const Point& p) {
        pos = p;
    }
};