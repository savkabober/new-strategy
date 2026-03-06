/*
Данные для метрики
*/
#pragma once

#include "../const.h"
#include "../objects/Circle.h"
#include "../objects/AbsRigBody.h"
#include "../objects/Rect.h"

struct MetricsData {
    int n;
    Circle objects[2 * MAX_ROBOT_COUNT + 1];
}