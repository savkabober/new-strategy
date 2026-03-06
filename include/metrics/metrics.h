/*
Метрики для алгоритма оптимизации
*/
#pragma once

#include "../aux/vecAux.h"
#include "MetricsData.h"
#include <ve

using namespace std;

namespace metrics
{
    // просчитать все участки езды
    void countSections(unsigned n, const double *x, void *data)
    {
        MetricsData *params = static_cast<MetricsData *>(data);
        bool compFlag = true;
        for (int i = 0; i < n; i++)
        {
            if (params->x[i] != x[i])
            {
                compFlag = false;
                break;
            }
        }
        if (!compFlag)
        {
            double dT, tMax, nMax = 0;
            params->r[0] = params->pos;
            params->v[0] = params->vel;
            params->t[0] = 0;
            for (int i = 0; i < n / 2; i++)
            {
                nMax++;
                params->v[nMax] = Point(MAX_VEL * cos(x[2 * i]), MAX_VEL * sin(x[2 * i]));
                tMax = (v[nMax] - v[nMax - 1]).mag() / MAX_ACC;
                if (tMax == 0)
                    params->a[nMax - 1] = Point(MAX_ACC, 0);
                else
                    params->a[nMax - 1] = (v[nMax] - v[nMax - 1]).unity() * MAX_ACC;
                dT = min(tMax, x[2 * i + 1]);
                params->t[nMax] = t[nMax - 1] + dT;
                params->r[nMax] = r[nMax - 1] + v[nMax - 1] * dT + a[nMax - 1] * dT * dT / 2;
                if (x[2 * i + 1] > tMax)
                {
                    nMax++;
                    params->a[nMax - 1] = Point();
                    dT = x[2 * i + 1] - tMax;
                    params->t[nMax - 1] = t[nMax - 1] + dT;
                    params->v[nMax] = v[nMax - 1];
                    params->r[nMax] = r[nMax - 1] + v[nMax - 1] * dT;
                }
            }
            params->nMax = nMax;
        }
    }
    // Время проезда - то, что минимизируем
    double minimizing(unsigned n, const double *x, double *grad, void *data)
    {
        countSections(n, x, data);
        MetricsData *params = static_cast<MetricsData *>(data);
        Point pos, vel, *a = params->a, *v = params->v, *r = params->r;
        double result = 0, prod[4], intersection, rad, *t = params->t;
        int nMax = params->nMax, nProd;
        bool isIn;
        for (int i = 0; i < params->nEnemies; i++)
        {
            rad = params->enemies[i].getRad();
            pos = params->enemies[i].getPos();
            vel = params->enemies[i].getVel();
            if ((pos - r[0]).mag() < rad) {
                isIn = true;
                intersection = 0;
            }
            else {
                isIn = false;
            }
            for (int j = 0; j < nMax; j++)
            {
                nProd = vecAux::parabolaCircleIntersection(prod, rad + params->safeDist, a[i], v[i] - vel, r[i] - pos);
                for (int k; k < nProd; k++)
                {
                    if (prod[k] < 0)
                        continue;
                    if (prod[k] > t[i + 1] - t[i])
                        break;
                    if (!isIn)
                    {
                        isIn = true;
                        intersection = prod[k] + t[i];
                    }
                    else
                    {
                        isIn = false;
                        result += prod[k] + t[i] - intersection;
                    }
                }
            }
        }
        result += t[nMax];
        return result;
    }

    // Ограничения по конечной точке, скорости, коллизии с роботами
    void constraints(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data)
    {
        countSections(n, x, data);
        MetricsData *params = static_cast<MetricsData *>(data);
        Point pos, vel, *a = params->a, *v = params->v, *r = params->r;
        double prod[4], intersection, rad, *t = params->t;
        int nMax = params->nMax, nProd;
        bool isIn;
        result[0] = (r[nMax] - params->endPos).mag() / MAX_VEL / MAX_VEL * MAX_ACC;
        result[1] = (v[nMax] - params->endVel).mag() / MAX_VEL;
        result[2] = 0;
        for (int i = 0; i < params->nEnemies; i++)
        {
            rad = params->enemies[i].getRad();
            pos = params->enemies[i].getPos();
            vel = params->enemies[i].getVel();
            if ((pos - r[0]).mag() < rad) {
                continue;
            }
            isIn = false;
            for (int j = 0; j < nMax; j++)
            {
                nProd = vecAux::parabolaCircleIntersection(prod, rad, a[i], v[i] - vel, r[i] - pos);
                for (int k; k < nProd; k++)
                {
                    if (prod[k] < 0)
                        continue;
                    if (prod[k] > t[i + 1] - t[i])
                        break;
                    if (!isIn)
                    {
                        isIn = true;
                        intersection = prod[k] + t[i];
                    }
                    else
                    {
                        isIn = false;
                        result[2] += prod[k] + t[i] - intersection;
                    }
                }
            }
        }
        result[2] * MAX_ACC / MAX_VEL;
    }
}