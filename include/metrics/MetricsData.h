/*
Данные для метрики
*/
#pragma once

#include "../const.h"
#include "../objects/Circle.h"
#include "../objects/AbsRigBody.h"
#include "../objects/Rect.h"
#include "../objects/Robot.h"

struct MetricsData {
    Point pos, vel, endPos, endVel;
    int nEnemies;
    AbsRigBody enemies[MAX_ROBOT_COUNT];
    double *t, *x;
    Point *a, *v, *r;
    int nMax;
    double safeDist;
}