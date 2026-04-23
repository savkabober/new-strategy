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
    void countSections(int n, MetricsData *data)
    {
        Point *a = data->a, *v = data->v, *r = data->r, deltaR, *vMax = data->vMax;
        double *t = data->t, *tMax = data->tMax, prod[4], dMag, *x = data->x;
        bool *isLong = data->isLong;
        int nProd;
        r[0] = data->pos;
        v[0] = data->vel;
        t[0] = 0;
        for (int i = 0; i < n / 2; i++)
        {
            vMax[i] = Point(cos(x[2 * i]) * MAX_VEL, sin(x[2 * i]) * MAX_VEL);
            dMag = (vMax[i] - v[i]).mag();
            tMax[i] = dMag / MAX_ACC;
            // cout << "AAAAA: " << tMax[i] << endl;
            if (tMax[i] == 0)
                a[i] = Point(MAX_ACC, 0);
            else
            {
                a[i] = (vMax[i] - v[i]) / dMag * MAX_ACC;
            }
            if (tMax[i] > x[2 * i + 1])
            {
                isLong[i] = false;
                t[2 * i + 1] = t[2 * i + 2] = t[2 * i] + x[2 * i + 1];
                v[i + 1] = v[i] + a[i] * x[2 * i + 1];
                r[2 * i + 1] = r[2 * i + 2] = r[2 * i] + v[i] * x[2 * i + 1] + a[i] * x[2 * i + 1] * x[2 * i + 1] / 2;
            }
            else
            {
                isLong[i] = true;
                t[2 * i + 1] = t[2 * i] + tMax[i];
                v[i + 1] = vMax[i];
                r[2 * i + 1] = r[2 * i] + v[i] * tMax[i] + a[i] * tMax[i] * tMax[i] / 2;
                t[2 * i + 2] = t[2 * i + 1] + x[2 * i + 1] - tMax[i];
                r[2 * i + 2] = r[2 * i + 1] + v[i + 1] * (x[2 * i + 1] - tMax[i]);
            }
        }
        deltaR = (data->endPos - r[n]);
        nProd = numAux::solveEq(prod, MAX_ACC * MAX_ACC, 0, -4 * v[n / 2].mag2(), 8 * (v[n / 2] ^ deltaR), -4 * deltaR.mag2());
        for (int i = 0; i < nProd; i++)
        {
            if (prod[i] >= 0)
            {
                prod[0] = prod[i];
                break;
            }
        }
        t[n + 1] = t[n + 2] = t[n] + prod[0];
        a[n / 2] = (deltaR - v[n / 2] * prod[0]) * 2 / (prod[0] * prod[0]);
        v[n / 2 + 1] = v[n / 2] + a[n / 2] * prod[0]; // = 2 * deltaR / prod[0] - v[n / 2];
        r[n + 1] = r[n + 2] = data->endPos;
        isLong[n / 2] = false;
    }
    // Просчитать все пересечения с препятствиями
    double countIntersections(int n, MetricsData *data, bool doSafe)
    {
        Point pos, vel, *a = data->a, *v = data->v, *r = data->r;
        double result = 0, prod[4], intersection, rad, *t = data->t, *tInt;
        if (doSafe)
            tInt = data->tIntMin;
        else
            tInt = data->tIntCon;
        int nProd, nowIdx;
        bool isIn;
        for (int i = 0; i < data->nEnemies; i++)
        {
            nowIdx = 0;
            rad = data->enemies[i].getRad();
            pos = data->enemies[i].getPos();
            vel = data->enemies[i].getVel();
            if (doSafe)
            {
                if ((pos - r[0]).mag() < rad + data->safeDist)
                {
                    isIn = true;
                    intersection = 0;
                    tInt[6 * (n / 2 + 1) * i] = 0;
                    nowIdx++;
                    cout << "huiDFSEJNIEGOWRGOWRGVERNV" << endl;
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

            for (int j = 0; j < n / 2 + 1; j++)
            {
                if (j % 2 && t[j] < 0)
                    continue;
                nProd = vecAux::parabolaCircleIntersection(prod, rad + data->safeDist * (doSafe), a[j], v[j] - vel, r[2 * j] - pos);
                for (int k = 0; k < nProd; k++)
                {
                    if (prod[k] < 0)
                        continue;
                    if (prod[k] >= t[2 * j + 1] - t[2 * j])
                        break;
                    tInt[6 * (n / 2 + 1) * i + 6 * j + nowIdx] = prod[k];
                    nowIdx++;
                    if (!isIn)
                        intersection = prod[k] + t[2 * j];
                    else
                        result += prod[k] + t[2 * j] - intersection;
                    isIn = !isIn;
                }
                for (; nowIdx < 4; nowIdx++)
                    tInt[6 * (n / 2 + 1) * i + 6 * j + nowIdx] = -1;
                nowIdx = 0;
                if (data->isLong[j])
                {
                    nProd = vecAux::lineCircleIntersection(prod, rad + data->safeDist * (doSafe), v[j] - vel, r[2 * j + 1] - pos);
                    for (int k = 0; k < nProd; k++)
                    {
                        if (prod[k] < 0)
                            continue;
                        if (prod[k] >= t[2 * j + 2] - t[2 * j + 1])
                            break;
                        tInt[6 * (n / 2 + 1) * i + 6 * j + nowIdx + 4] = prod[k];
                        nowIdx++;
                        if (!isIn)
                            intersection = prod[k] + t[2 * j + 1];
                        else
                            result += prod[k] + t[2 * j + 1] - intersection;
                        isIn = !isIn;
                    }
                    for (; nowIdx < 2; nowIdx++)
                        tInt[6 * (n / 2 + 1) * i + 6 * j + nowIdx + 4] = -1;
                    nowIdx = 0;
                }
                else
                    tInt[6 * (n / 2 + 1) * i + 6 * j + 4] = tInt[6 * (n / 2 + 1) * i + 6 * j + 5] = -1;
            }
        }
        return result;
    }
    // Функция просчета цепочки
    double chainGrad(int n, int i, bool shortFl, MetricsData *data)
    {
        Point *dV = data->dV, *dR = data->dR, *a = data->a, *v = data->v, *r = data->r, aNormal, deltaR, *dA = data->dA;
        double *t = data->t, *tMax = data->tMax, deltaT, dP, dT;
        bool shortFlag = shortFl;
        // В данный момент просчитаны все участки езды. Начнем брать производную по времени.
        for (int j = i + 1; j < n / 2 && !shortFlag; j++)
        {
            // А здесь мы запускаем цепочку - от элемента который меняется и до конца
            // Важно: если мы дошли до элемента, где есть РПД, мы победили: дальше на скорости цепочка не распространяется,
            // а производная перемещения сохраняется
            if (data->isLong[j])
            {
                dT = -(dV[j] ^ a[j]) / (MAX_ACC * MAX_ACC);
                dR[2 * j + 1] = dR[2 * j] + (v[j] + v[j + 1]) * dT / 2 + dV[j] * (t[2 * j + 1] - t[2 * j]) / 2;
                dR[2 * j + 2] = dR[2 * j + 1] - v[j + 1] * dT;
                dV[j + 1] = 0;
                for (int k = j + 1; k < n / 2; k++)
                {
                    dR[2 * k + 2] = dR[2 * k + 1] = dR[2 * k];
                    dV[k + 1] = dV[k];
                }
                shortFlag = true;
            }
            else
            {
                aNormal = Point(-a[j].y, a[j].x);
                dV[j + 1] = (a[j] * (dV[j] ^ a[j]) + aNormal * (dV[j] ^ aNormal) * (tMax[j] - t[2 * j + 2] + t[2 * j]) / tMax[j]) / (MAX_ACC * MAX_ACC);
                dR[2 * j + 2] = dR[2 * j + 1] = dR[2 * j] + (dV[j] + dV[j + 1]) * (t[2 * j + 2] - t[2 * j]) / 2;
                // if (i == 0 && j == 2) {
                //     cout << (dV[j] + dV[j + 1]) * (t[2 * j + 2] - t[2 * j]) / 2 << " tudoooo" << endl;
                //     //cout << dV[j] << " " << dV[j + 1] << " " << t << endl;
                // }
                // cout << "kakat: " << j << endl;
            }
            // cout << "iter: " << j << dR[6] << endl;
        }
        // мы просчитали цепочку, теперь финальный шаг - найти градиент (внезапно)
        deltaR = data->endPos - r[n];
        deltaT = t[n + 2] - t[n];
        dP = 4 * MAX_ACC * MAX_ACC * deltaT * deltaT * deltaT - 8 * v[n / 2].mag2() * deltaT + 8 * (v[n / 2] ^ deltaR);
        dT = -(v[n / 2] ^ dV[n / 2]) * deltaT * deltaT + deltaT * ((dV[n / 2] ^ deltaR) - (v[n / 2] ^ dR[n]));
        dT += (deltaR ^ dR[n]);
        dT *= -8 / dP;
        dV[n / 2 + 1] = -dR[n] * 2 / deltaT - deltaR * 2 / (deltaT * deltaT) * dT - dV[n / 2];
        dR[n + 1] = 0;
        for (int j = i; j < n / 2 + 1; j++)
        {
            aNormal = Point(-a[j].y, a[j].x);
            dA[j] = aNormal * (aNormal ^ (dV[j + 1] - dV[j])) / (t[2 * j + 1] - t[2 * j]) / (MAX_ACC * MAX_ACC);
        }
        return dT;
    }

    bool isRepeated(int n, const double *x, MetricsData *data)
    {
        bool compFlag = true;
        for (int i = 0; i < n; i++)
        {
            if (data->x[i] != x[i])
            {
                compFlag = false;
                break;
            }
        }
        for (int i = 0; i < n; i++)
        {
            data->x[i] = x[i];
        }
        return false;
        return compFlag;
    }

    double intGrad(int n, MetricsData *data, int i, bool doSafe)
    {
        Point *dA = data->dA, vNormal, *v = data->v, *a = data->a, aNormal, vel, pos, *dV = data->dV, *r = data->r, *dR = data->dR;
        bool isIn = false;
        double dT, dP, *tInt, result = 0, t;
        if (doSafe)
        {
            tInt = data->tIntMin;
        }
        else
        {
            tInt = data->tIntCon;
        }
        for (int j = i; j < n / 2 + 1; j++)
        {
            for (int k = 0; k < data->nEnemies; k++)
            {
                vel = v[j] - data->enemies[k].getVel();
                pos = r[2 * j] - data->enemies[k].getPos();
                for (int s = 0; s < 4; s++)
                {
                    t = tInt[6 * (n / 2 + 1) * k + 6 * j + s];
                    if (t < 0)
                        break;
                    dP = MAX_ACC * MAX_ACC * t * t * t;
                    dP += 3 * (a[j] ^ vel) * t * t;
                    dP += 2 * t * ((a[j] ^ pos) + vel.mag2());
                    dP += 2 * (vel ^ pos);
                    dT = ((dA[j] ^ vel) + (a[j] ^ dV[j])) * t * t * t;
                    dT += (((dA[j] ^ pos) + (a[j] ^ dR[2 * j])) + 2 * (vel ^ dV[j])) * t * t;
                    dT += 2 * ((dV[j] ^ pos) + (vel ^ dR[2 * j])) * t;
                    dT += 2 * (pos ^ dR[2 * j]);
                    dT *= -1 / dP;
                    if (!isIn)
                        result -= dT;
                    else
                        result += dT;
                    isIn = !isIn;
                }
                if (data->isLong[j])
                {
                    vel = v[j + 1] - data->enemies[k].getVel();
                    pos = r[2 * j + 1] - data->enemies[k].getPos();
                    for (int s = 0; s < 2; s++)
                    {
                        t = tInt[6 * (n / 2 + 1) * k + 6 * j + s + 4];
                        if (t < 0)
                            break;
                        dP = 2 * t * (vel.mag2());
                        dP += 2 * (vel ^ pos);
                        dT = 2 * (vel ^ dV[j + 1]) * t * t;
                        dT += 2 * ((dV[j + 1] ^ pos) + (vel ^ dR[2 * j + 1])) * t;
                        dT += 2 * (pos ^ dR[2 * j + 1]);
                        dT *= -1 / dP;
                        if (!isIn)
                            result -= dT;
                        else
                            result += dT;
                        isIn = !isIn;
                    }
                }
            }
        }
        return result;
    }

    void mainActions(int n, const double *x, MetricsData *data)
    {
        Point vNormal, *vMax = data->vMax, *a = data->a, *dV = data->dV, *dR = data->dR, *v = data->v;
        double dT, *t = data->t, *tMax = data->tMax;
        bool shortFlag;
        if (isRepeated(n, x, data))
            return;
        countSections(n, data);
        data->resultCon[0] = (v[n / 2 + 1] - data->endVel).x / MAX_VEL;
        data->resultCon[1] = (v[n / 2 + 1] - data->endVel).y / MAX_VEL;
        data->resultCon[2] = countIntersections(n, data, false) * (MAX_ACC / MAX_VEL);
        data->resultMin = (countIntersections(n, data, true) + t[n + 2]) * MAX_ACC / MAX_VEL;
        for (int i = 0; i < n / 2; i++)
        {
            dV[i] = dR[2 * i] = 0;
            vNormal = Point(-vMax[i].y, vMax[i].x);
            if (data->isLong[i])
            {
                dT = (a[i] ^ vNormal) / (MAX_ACC * MAX_ACC);
                dV[i + 1] = vNormal;
                dR[2 * i + 1] = dV[i + 1] * (t[2 * i + 1] - t[2 * i]) / 2 + (v[i] + v[i + 1]) * dT / 2;
                dR[2 * i + 2] = dR[2 * i + 1] + dV[i + 1] * (t[2 * i + 2] - t[2 * i + 1]) - v[i + 1] * dT;
            }
            else
            {
                dV[i + 1] = vNormal * (t[2 * i + 1] - t[2 * i]) / tMax[i];
                dR[2 * i + 1] = dR[2 * i + 2] = dV[i + 1] * (t[2 * i + 1] - t[2 * i]) / 2;
            }
            dT = chainGrad(n, i, false, data);
            data->gradCon[2 * i] = data->dV[n / 2 + 1].x / MAX_VEL;
            data->gradCon[n + 2 * i] = data->dV[n / 2 + 1].y / MAX_VEL;
            data->gradMin[2 * i] = dT;
            dT = intGrad(n, data, i, false);
            data->gradCon[2 * n + 2 * i] = dT;
            dT = intGrad(n, data, i, true);
            data->gradMin[2 * i] += dT;
            data->gradCon[2 * n + 2 * i] *= (MAX_ACC / MAX_VEL);
            data->gradMin[2 * i] *= (MAX_ACC / MAX_VEL);
        }
        for (int i = 0; i < n / 2; i++)
        {
            dV[i] = dR[2 * i] = 0;
            if (data->isLong[i])
            {
                shortFlag = true;
                dR[2 * i + 1] = 0;
                dR[2 * i + 2] = v[i + 1];
                dV[i + 1] = 0;
                for (int j = i + 1; j < n / 2; j++)
                {
                    dV[j + 1] = dV[j];
                    dR[2 * j + 1] = dR[2 * j + 2] = dR[2 * j];
                }
            }
            else
            {
                shortFlag = false;
                dR[2 * i + 1] = dR[2 * i + 2] = v[i + 1];
                dV[i + 1] = a[i];
            }
            dT = chainGrad(n, i, shortFlag, data);
            data->gradCon[2 * i + 1] = data->dV[n / 2 + 1].x / MAX_VEL;
            data->gradCon[n + 2 * i + 1] = data->dV[n / 2 + 1].y / MAX_VEL;
            data->gradMin[2 * i + 1] = dT + 1;
            dT = intGrad(n, data, i, false);
            data->gradCon[2 * n + 2 * i + 1] = dT;
            for (int j = 0; j < data->nEnemies; j++)
            {
                if ((data->enemies[j].getPos() - data->r[2 * i + 2]).mag() < data->enemies[j].getRad())
                    data->gradCon[2 * n + 2 * i + 1] += 1;
            }
            dT = intGrad(n, data, i, true);
            data->gradMin[2 * i + 1] += dT;
            for (int j = 0; j < data->nEnemies; j++)
            {
                if ((data->enemies[j].getPos() - data->r[2 * i + 2]).mag() < data->enemies[j].getRad() + data->safeDist)
                    data->gradMin[2 * i + 1] += 1;
            }
            data->gradCon[2 * n + 2 * i + 1] *= (MAX_ACC / MAX_VEL);
            data->gradMin[2 * i + 1] *= (MAX_ACC / MAX_VEL);
            // if (i == 0) {
            //     cout << "bebe: " << dV[5] << " " << (dV[2] + dV[3]) * (t[6] - t[4]) / 2 << endl;
            //     // cout << dV[2] << " " << dV[3] << endl;
            // }
        }
    }

    // Время проезда - то, что минимизируем
    double minimizing(unsigned n, const double *x, double *grad, void *voidData)
    {
        MetricsData *data = static_cast<MetricsData *>(voidData);
        mainActions(n, x, data);
        for (int i = 0; i < int(n); i++)
        {
            grad[i] = data->gradMin[i];
        }
        return data->resultMin;
    }

    // Ограничения по конечной точке, скорости, коллизии с роботами
    void constraints(unsigned int m, double *result, unsigned n, const double *x, double *grad, void *voidData)
    {
        MetricsData *data = static_cast<MetricsData *>(voidData);
        mainActions(n, x, data);
        result[0] = data->resultCon[0];
        result[1] = data->resultCon[1];
        result[2] = data->resultCon[2];
        for (int i = 0; i < 3 * int(n); i++)
        {
            grad[i] = data->gradCon[i];
        }
    }
}