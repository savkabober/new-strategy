/*
Метрики для алгоритма оптимизации
*/
#pragma once

#include "../auxiliary/vecAux.h"
#include "MetricsData.h"
#include <iostream>

using namespace std;

namespace metrics
{
    // Просчитать все участки езды
    void countSections(unsigned n, const double *x, void *data)
    {
        MetricsData *params = static_cast<MetricsData *>(data);
        Point *a = params->a, *v = params->v, *r = params->r, deltaR, vMax;
        double *t = params->t, *tMax = params->tMax, prod[4], dMag;
        int nProd;
        bool compFlag = true;
        for (unsigned int i = 0; i < n; i++)
        {
            if (params->x[i] != x[i])
            {
                compFlag = false;
                break;
            }
        }
        cout << "compFlag: " << compFlag << endl;
        for (unsigned int i = 0; i < n; i++)
        {
            params->x[i] = x[i];
        }
        compFlag = 0;
        if (!compFlag)
        {
            r[0] = params->pos;
            v[0] = params->vel;
            t[0] = 0;
            for (unsigned int i = 0; i < n / 2; i++)
            {
                vMax = Point(cos(x[2 * i]) * MAX_VEL, sin(x[2 * i]) * MAX_VEL);
                dMag = (vMax - v[i]).mag();
                tMax[0] = dMag/MAX_ACC;
                // cout << "AAAAA: " << tMax[i] << endl;
               if (tMax[i] == 0)
                    a[i] = Point(MAX_ACC, 0);
                else
                {
                    a[i] = (vMax - v[i]) / dMag * MAX_ACC;
                }
                if (tMax[i] > x[2 * i + 1])
                {
                    t[2 * i + 1] = -1;
                    t[2 * i + 2] = t[2 * i] + x[2 * i + 1];
                    v[i + 1] = v[i] + a[i] * x[2 * i + 1];
                    r[2 * i + 2] = r[2 * i] + v[i] * x[2 * i + 1] + a[i] * x[2 * i + 1] * x[2 * i + 1] / 2;
                }
                else
                {
                    t[2 * i + 1] = t[2 * i] + tMax[i];
                    v[i + 1] = vMax;
                    r[2 * i + 1] = r[2 * i] + v[i] * tMax[i] + a[i] * tMax[i] * tMax[i] / 2;
                    t[2 * i + 2] = t[2 * i + 1] + x[2 * i + 1] - tMax[i];
                    r[2 * i + 2] = r[2 * i + 1] + v[i + 1] * (x[2 * i + 1] - tMax[i]);
                }
            }
            deltaR = (params->endPos - r[n]);
            params->vMag = v[n / 2].mag();
            params->rMag = deltaR.mag();
            nProd = numAux::solveEq(prod, MAX_ACC * MAX_ACC, 0, -4 * params->vMag * params->vMag, 8 * (v[n / 2] ^ deltaR), -4 * params->rMag * params->rMag);
            for (int i = 0; i < nProd; i++)
            {
                if (prod[i] >= 0)
                {
                    prod[0] = prod[i];
                    break;
                }
            }
            t[n + 2] = t[n] + prod[0];
            t[n + 1] = -1;
            a[n / 2] = (deltaR - v[n / 2] * prod[0]) * 2 / (prod[0] * prod[0]);
            v[n / 2 + 1] = v[n / 2] + a[n / 2] * prod[0]; // = 2 * deltaR / prod[0] - v[n / 2];
            r[n + 2] = params->endPos;
        }
    }
    // Просчитать все пересечения с препятствиями
    double countIntersections(unsigned n, const double *x, double *grad, void *data, bool doSafe)
    {
        MetricsData *params = static_cast<MetricsData *>(data);
        Point pos, vel, *a = params->a, *v = params->v, *r = params->r;
        double result = 0, prod[4], intersection, rad, *t = params->t;
        int nProd;
        bool isIn;
        for (int i = 0; i < params->nEnemies; i++)
        {
            rad = params->enemies[i].getRad();
            pos = params->enemies[i].getPos();
            vel = params->enemies[i].getVel();
            if (doSafe)
            {
                if ((pos - r[0]).mag() < rad + params->safeDist)
                {
                    isIn = true;
                    intersection = 0;
                }
                else
                    isIn = false;
            }
            else
            {
                if ((pos - r[0]).mag() < rad)
                    continue;
                isIn = false;
            }

            for (unsigned int j = 0; j < n + 2; j++)
            {
                if (j % 2 && t[j] < 0)
                    continue;
                nProd = vecAux::parabolaCircleIntersection(prod, rad + params->safeDist * (doSafe), a[j], v[j] - vel, r[j] - pos);
                for (int k = 0; k < nProd; k++)
                {
                    if (prod[k] < 0)
                        continue;
                    if (prod[k] > t[j + 1] - t[j])
                        break;
                    if (!isIn)
                    {
                        isIn = true;
                        intersection = prod[k] + t[j];
                    }
                    else
                    {
                        isIn = false;
                        result += prod[k] + t[j] - intersection;
                    }
                }
            }
        }
        return result;
    }
    // Функция просчета цепочки
    void chainGrad(unsigned n, int i, void *data)
    {
        MetricsData *params = static_cast<MetricsData *>(data);
        Point *dV = params->dV, *dR = params->dR, *a = params->a, *v = params->v, *r = params->r, aNormal, deltaR;
        double *t = params->t, *tMax = params->tMax, deltaT, dP, dT;
        bool shortFlag = false;
        // В данный момент просчитаны все участки езды. Начнем брать производную по времени.
        if (t[2 * i + 1] < 0)
        {
            dV[i + 1] = a[i];
            dR[i + 1] = v[i + 1];
        }
        else
        {
            shortFlag = true;
            dR[n / 2] = v[i + 1];
            dV[n / 2] = 0;
        }
        for (unsigned int j = i + 1; j < n / 2 && !shortFlag; j++)
        {
            // А здесь мы запускаем цепочку - от элемента который меняется и до конца
            // Важно: если мы дошли до элемента, где есть РПД, мы победили: дальше на скорости цепочка не распространяется,
            // а производная перемещения сохраняется
            if (t[2 * j + 1] < 0)
            {
                aNormal = Point(-a[j].y, a[j].x);
                dV[j + 1] = (a[j] * (dV[j] ^ a[j]) + aNormal * (dV[j] ^ aNormal) * (tMax[j] - t[2 * j + 2] + t[2 * j]) / tMax[j]) / (MAX_ACC * MAX_ACC);
                dR[j + 1] = dR[j] + (dV[j] + dV[j + 1]) * (t[2 * j * 2] - t[2 * j]) / 2;
            }
            else
            {
                dR[n / 2] = dR[j] + (v[j + 1] - v[j]) * (dV[j] ^ a[j]) / (MAX_ACC * MAX_ACC) / 2 + dV[j] * (t[2 * j + 1] - t[2 * j]) / 2;
                dV[n / 2] = 0;
                shortFlag = true;
            }
        }
        // мы просчитали цепочку, теперь финальный шаг - найти градиент (внезапно)
        deltaR = params->endPos - r[n];
        deltaT = t[n + 2] - t[n];
        dP = 4 * MAX_ACC * MAX_ACC * deltaT * deltaT * deltaT - 8 * params->vMag * params->vMag * deltaT + 8 * (v[n / 2] ^ deltaR);
        dT = -(-8 * (v[n / 2] ^ dV[n / 2]) * deltaT * deltaT + 8 * deltaT * ((dV[n / 2] ^ deltaR) - (v[n / 2] ^ dR[n / 2])) + 8 * (deltaR ^ dR[n / 2])) / dP;
        dV[n / 2 + 1] = -dR[n / 2] *2 / deltaT - deltaR *2/ (deltaT * deltaT) * dT - dV[n / 2];
    }

    // Время проезда - то, что минимизируем
    double minimizing(unsigned n, const double *x, double *grad, void *data)
    {
        countSections(n, x, data);
        MetricsData *params = static_cast<MetricsData *>(data);
        Point pos, vel;
        double result, *t = params->t;
        result = countIntersections(n, x, grad, data, true);
        result += t[n + 2];
        return result;
    }

    // Ограничения по конечной точке, скорости, коллизии с роботами
    void constraints(unsigned int m,double *result, unsigned n, const double *x, double *grad, void *data)
    {
        countSections(n, x, data);
        MetricsData *params = static_cast<MetricsData *>(data);
        Point pos, vel, *v = params->v, aNormal;
        result[0] = (v[n / 2 + 1] - params->endVel).x / MAX_VEL;
        result[1] = (v[n / 2 + 1] - params->endVel).x / MAX_VEL;
        result[2] = countIntersections(n, x, grad, data, false);
        result[2] *= MAX_ACC / MAX_VEL;
    }
}