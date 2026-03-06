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
    Robot(Point p = Point(), double r = ROBOT_R, Point v = Point(), int i = 0, int t = 0) : AbsRigBody(p, r, v), id(i), team(t) {}
    //геттеры
    int getId() const {
        return id;
    }
    int getTeam() const {
        return team;
    }
    int getRegime() const {
        return regime;
    }
    int getActive() const {
        return active;
    }
    Point getControl() const {
        return control;
    }
    //сеттеры
    void setId(int i) {
        id = i;
    }
    void setTeam(int t) {
        team = t;
    }
    void setRegime(int r) {
        regime = r;
    }
    void setActive(int a) {
        active = a;
    }
    void setControl(const Point& c) {
        control = c;
    }
};