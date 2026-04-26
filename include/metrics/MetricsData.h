/*
Данные для метрики
*/
#pragma once

#include "../const.h"
#include "../objects/Circle.h"
#include "../objects/AbsRigBody.h"
#include "../objects/Robot.h"

struct MetricsData {
    Point pos, vel, endPos, endVel;
    int nEnemies, n;
    AbsRigBody enemies[MAX_ROBOT_COUNT];
    double *t, *x, *tMax, *gradCon, *gradMin, *tIntCon, *tIntMin, *resultCon, resultMin, safeDist, *dT;
    bool *isLong;
    Point *a, *v, *r, *vMax, *dV, *dR, *dA;
};