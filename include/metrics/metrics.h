/*
Метрики для алгоритма оптимизации
*/
#pragma once

#include "../aux/vecAux.h"
#include "MetricsData.h"

using namespace std;

namespace metrics {
    //Время проезда - то, что минимизируем
    double minimizing(unsigned n, const double *x, double *grad, void *data) {
        double summ = 0;
        for (int i = 0; i <= x.n / 2; i++) {
            grad[2 * i] = 0;
            grad[2 * i + 1] = x[2 * i + 1];
            summ += x[2 * i + 1];
        }
        return summ;
    }

    //Ограничения по конечной точке, скорости, коллизии с роботами
    void constraints(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data) {
        MetricsData* params = static_cast<MetricsData*>(data);
        Point p[n + 1], v[n + 1], a[n];
        double t[n + 1];
        double tMax, cosAlpha, nMax = 1;
        r[0] = Point(params[0], params[1]);
        v[0] = Point(params[2], params[3]);
        for (int i = 0; i < n / 2; i++) {
            v[nMax] = Point(MAX_VEL * cos(x[2 * i]), MAX_VEL * sin(x[2 * i]));
            tMax = (v[nMax] - v[nMax - 1]).mag() / MAX_ACC;
            if (tMax == 0) {
                a[nMax - 1] = Point(MAX_ACC, 0);
            }
            else {
                a[nMax - 1] = (v[nMax] - v[nMax - 1]).unity() * MAX_ACC;
            }
            if (tMax > abs(x[2 * i + 1])) {
                r[nMax] = p[nMax - 1] + (v[nMax - 1] + v[nMax]) / 2 * tMax;
                nMax++;
                v[nMax] = v[nMax - 1];
                r[nMax] = p[nMax - 1] + v[nMax] * (abs(x[2 * i + 1]) - tMax);
            }
            else {
                p[nMax] = p[nMax - 1] + (v[nMax - 1] + v[nMax]) / 2 * abs(x[2 * i + 1]);
            }
            nMax++;
        }
        nMax--;
        result[0] = (r[nMax] - Point[params[4], params[5]]).mag() / MAX_VEL;
        result[1] = (v[nMax] - Point(params[6] - params[7])).mag() / MAX_ACC;
    }
}