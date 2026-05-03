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
    double t[2 * N_SOLVER_PAIRS + 4], x[2 * N_SOLVER_PAIRS], tMax[N_SOLVER_PAIRS], gradCon[6 * N_SOLVER_PAIRS], gradMin[2 * N_SOLVER_PAIRS], tIntCon[6 * (N_SOLVER_PAIRS + 1) * MAX_ROBOT_COUNT], tIntMin[6 * (N_SOLVER_PAIRS + 1) * MAX_ROBOT_COUNT], resultCon[3], resultMin, dT[2 * N_SOLVER_PAIRS + 4];
    bool isLong[N_SOLVER_PAIRS + 1];
    Point a[N_SOLVER_PAIRS + 1], v[N_SOLVER_PAIRS + 2], r[2 * N_SOLVER_PAIRS + 4], vMax[N_SOLVER_PAIRS], dV[N_SOLVER_PAIRS + 2], dR[2 * N_SOLVER_PAIRS + 4], dA[N_SOLVER_PAIRS + 1];
};