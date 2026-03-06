/*
Класс с роботом
*/
#pragma once

#include "AbsRigBody.h"
#include "../const.h"

class Robot : public AbsRigBody {
    protected:
    int id;
    int team;
    int regime;
    int active;
    Point control;
    public:
    Robot(Point p = Point(), double r = ROBOT_R, Point v = Point(), int id = 0, int t = 0) : AbsRigBody(p, r, v), r_id(id), team(t) {}
    //геттеры
    int get_id() const {
        return id;
    }
    int get_team() const {
        return team;
    }
    int get_regime() const {
        return regime;
    }
    int get_active() const {
        return active;
    }
    Point get_control() const {
        return control;
    }
    //сеттеры
    void set_id(int i) {
        id = i;
    }
    void set_team(int t) {
        team = t;
    }
    void set_regime(int r) {
        regime = r;
    }
    void set_active(int a) {
        active = a;
    }
    void set_control(const Point& c) {
        control = c;
    }
};