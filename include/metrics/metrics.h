/*
Метрики для алгоритма оптимизации
*/
#pragma once

#include "../auxiliary/vecAux.h"
#include "MetricsData.h"
#include <iostream>
#include "../Timer.h"
#include "../drawer.h"

using namespace std;

namespace metrics
{
    // Просчитать все участки езды
    void countSections(int n, MetricsData *data)
    {
        Point *a = data->a, *v = data->v, *r = data->r, deltaR, *vMax = data->vMax;
        double *t = data->t, *tMax = data->tMax, prod[4], *x = data->x;
        bool *isLong = data->isLong;
        int nProd;
        r[0] = data->pos;
        v[0] = data->vel;
        t[0] = 0;
        for (int i = 0; i < n / 2; i++)
        {
            vMax[i] = Point(cos(x[2 * i]) * MAX_VEL, sin(x[2 * i]) * MAX_VEL);
            tMax[i] = (vMax[i] - v[i]).mag() / MAX_ACC;
            if (tMax[i] == 0)
                a[i] = Point(-vMax[i].y, vMax[i].x) * MAX_ACC / MAX_VEL;
            else
            {
                a[i] = (vMax[i] - v[i]) / tMax[i];
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
        double deltaMag, deltaMagMin = -1;
        int idxMin = -1;
        for (int i = 0; i < nProd; i++)
        {
            if (prod[i] >= 0)
            {
                if (idxMin < 0)
                {
                    idxMin = i;
                    deltaMagMin = (data->endVel - deltaR * 2 / prod[i] + v[n / 2]).mag();
                }
                else
                {
                    deltaMag = (data->endVel - deltaR * 2 / prod[i] + v[n / 2]).mag();
                    if (deltaMag < deltaMagMin)
                    {
                        deltaMagMin = deltaMag;
                        idxMin = i;
                    }
                }
            }
        }
        isLong[n / 2] = false;
        v[n / 2 + 1] = data->endVel;
        t[n + 1] = t[n + 2] = t[n] + (v[n / 2 + 1] - v[n / 2]).mag() / MAX_ACC;
        a[n / 2] = (v[n / 2 + 1] - v[n / 2]) / (t[n + 1] - t[n]);
        r[n + 1] = r[n + 2] = r[n] + v[n / 2] * (t[n + 1] - t[n]) + a[n / 2] * (t[n + 1] - t[n]) * (t[n + 1] - t[n]) / 2;
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
                if ((pos - r[0]).mag() < rad + SAFE_DIST)
                {
                    isIn = true;
                    intersection = 0;
                    tInt[6 * (n / 2 + 1) * i] = 0;
                    nowIdx++;
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
                nProd = vecAux::parabolaCircleIntersection(prod, rad + SAFE_DIST * (doSafe), a[j], v[j] - vel, r[2 * j] - pos);
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
                    nProd = vecAux::lineCircleIntersection(prod, rad + SAFE_DIST * (doSafe), v[j + 1] - vel, r[2 * j + 1] - pos);
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
            if (isIn)
            {
                for (int j = 1; j <= 6 * (n / 2 + 1); j++)
                {
                    if (tInt[6 * (n / 2 + 1) * (i + 1) - j] >= 0)
                    {
                        tInt[6 * (n / 2 + 1) * (i + 1) - j] = -1;
                        break;
                    }
                }
            }
        }
        return result;
    }
    // Функция просчета цепочки
    void chainGrad(int n, int i, bool shortFl, MetricsData *data)
    {
        Point *dV = data->dV, *dR = data->dR, *a = data->a, *v = data->v, aNormal, deltaR, *dA = data->dA;
        double *t = data->t, *tMax = data->tMax, dTNow, *dT = data->dT;
        bool shortFlag = shortFl;
        // В данный момент просчитаны все участки езды. Начнем брать производную по времени.
        for (int j = i + 1; j < n / 2 && !shortFlag; j++)
        {
            // А здесь мы запускаем цепочку - от элемента который меняется и до конца
            // Важно: если мы дошли до элемента, где есть РПД, мы победили: дальше на скорости цепочка не распространяется,
            // а производная перемещения сохраняется
            if (data->isLong[j])
            {
                dTNow = -(dV[j] ^ a[j]) / (MAX_ACC * MAX_ACC);
                dR[2 * j + 1] = dR[2 * j] + (v[j] + v[j + 1]) * dTNow / 2 + dV[j] * (t[2 * j + 1] - t[2 * j]) / 2;
                dR[2 * j + 2] = dR[2 * j + 1] - v[j + 1] * dTNow;
                dV[j + 1] = 0;
                dT[2 * j + 1] = dT[2 * i + 2] + dTNow;
                dT[2 * j + 2] = dT[2 * i + 2];
                for (int k = j + 1; k < n / 2; k++)
                {
                    dT[2 * k + 2] = dT[2 * k + 1] = dT[2 * i + 2];
                    dR[2 * k + 2] = dR[2 * k + 1] = dR[2 * k];
                    dV[k + 1] = dV[k];
                }
                shortFlag = true;
            }
            else
            {
                dT[2 * j + 2] = dT[2 * j + 1] = dT[2 * i + 2];
                aNormal = Point(-a[j].y, a[j].x);
                dV[j + 1] = (a[j] * (dV[j] ^ a[j]) + aNormal * (dV[j] ^ aNormal) * (tMax[j] - t[2 * j + 2] + t[2 * j]) / tMax[j]) / (MAX_ACC * MAX_ACC);
                dR[2 * j + 2] = dR[2 * j + 1] = dR[2 * j] + (dV[j] + dV[j + 1]) * (t[2 * j + 2] - t[2 * j]) / 2;
            }
        }
        // мы просчитали цепочку, теперь финальный шаг - найти градиент (внезапно)
        dTNow = -(dV[n / 2] ^ a[n / 2]) / (MAX_ACC * MAX_ACC);
        dR[n + 1] = dR[n + 2] = dR[n] + (v[n / 2] + v[n / 2 + 1]) * dTNow / 2 + dV[n / 2] * (t[n + 1] - t[n]) / 2;
        dV[n / 2 + 1] = 0;
        dT[n + 1] = dT[n + 2] = dT[2 * i + 2] + dTNow;
        for (int j = i; j < n / 2 + 1; j++)
        {
            aNormal = Point(-a[j].y, a[j].x);
            dA[j] = aNormal * (aNormal ^ (dV[j + 1] - dV[j])) / (t[2 * j + 1] - t[2 * j]) / (MAX_ACC * MAX_ACC);
        }
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
        // return false;
        return compFlag;
    }

    double intGrad(int n, MetricsData *data, int i, bool doSafe)
    {
        Point *dA = data->dA, *v = data->v, *a = data->a, aNormal, vel, pos, *dV = data->dV, *r = data->r, *dR = data->dR;
        bool isIn = false;
        double dTNow, dP, *tInt, result = 0, t, *dT = data->dT;
        if (doSafe)
        {
            tInt = data->tIntMin;
        }
        else
        {
            tInt = data->tIntCon;
        }
        for (int k = 0; k < data->nEnemies; k++)
        {
            isIn = false;
            for (int j = 0; j < i; j++) {
                for (int s = 0; s < 6; s++) {
                    if (tInt[6 * (n / 2 + 1) * k + 6 * j + s] >= 0) {
                        isIn = !isIn;
                    }
                }
            }
            for (int j = i; j < n / 2 + 1; j++)
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
                    dTNow = ((dA[j] ^ vel) + (a[j] ^ dV[j])) * t * t * t;
                    dTNow += (((dA[j] ^ pos) + (a[j] ^ dR[2 * j])) + 2 * (vel ^ dV[j])) * t * t;
                    dTNow += 2 * ((dV[j] ^ pos) + (vel ^ dR[2 * j])) * t;
                    dTNow += 2 * (pos ^ dR[2 * j]);
                    dTNow *= -1 / dP;
                    if (!isIn)
                        result -= dTNow + dT[2 * j];
                    else
                        result += dTNow + dT[2 * j];
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
                        dTNow = 2 * (vel ^ dV[j + 1]) * t * t;
                        dTNow += 2 * ((dV[j + 1] ^ pos) + (vel ^ dR[2 * j + 1])) * t;
                        dTNow += 2 * (pos ^ dR[2 * j + 1]);
                        dTNow *= -1 / dP;
                        if (!isIn)
                            result -= dTNow + dT[2 * j + 1];
                        else
                            result += dTNow + dT[2 * j + 1];
                        isIn = !isIn;
                    }
                }
            }
        }
        return result;
    }

    void mainActions(int n, const double *x, MetricsData *data)
    {
        Point vNormal, dNormal, *vMax = data->vMax, *a = data->a, *dV = data->dV, *dR = data->dR, *v = data->v;
        double *t = data->t, *tMax = data->tMax, *dT = data->dT, dTNow;
        bool shortFlag;
        if (isRepeated(n, x, data))
            return;
        countSections(n, data);
        data->resultCon[0] = (data->r[n + 1] - data->endPos).x * MAX_ACC / (MAX_VEL * MAX_VEL);
        data->resultCon[1] = (data->r[n + 1] - data->endPos).y * MAX_ACC / (MAX_VEL * MAX_VEL);
        data->resultCon[2] = countIntersections(n, data, false) * (MAX_ACC / MAX_VEL);
        data->resultMin = (countIntersections(n, data, true) * K_INTERSECT + t[n + 2]/* + (data->r[n + 1] - data->endPos).mag2() / (MAX_VEL * MAX_VEL)*/) * MAX_ACC / MAX_VEL;
        for (int i = 0; i < n / 2; i++)
        {
            dV[i] = dR[2 * i] = Point(0, 0);
            dT[2 * i] = 0;
            vNormal = Point(-vMax[i].y, vMax[i].x);
            if (data->isLong[i])
            {
                dTNow = (a[i] ^ vNormal) / (MAX_ACC * MAX_ACC);
                dV[i + 1] = vNormal;
                dR[2 * i + 1] = dV[i + 1] * (t[2 * i + 1] - t[2 * i]) / 2 + (v[i] + v[i + 1]) * dTNow / 2;
                dR[2 * i + 2] = dR[2 * i + 1] + dV[i + 1] * (t[2 * i + 2] - t[2 * i + 1]) - v[i + 1] * dTNow;
                dT[2 * i + 1] = dTNow;
                dT[2 * i + 2] = 0;
            }
            else
            {
                dT[2 * i + 2] = dT[2 * i + 1] = 0;
                dNormal = Point(-(vMax[i].y - v[i].y), (vMax[i].x - v[i].x));
                dV[i + 1] = dNormal * (dNormal ^ vNormal) * (t[2 * i + 1] - t[2 * i]);
                dV[i + 1] /= (tMax[i] * tMax[i] * tMax[i] * MAX_ACC * MAX_ACC);
                dR[2 * i + 1] = dR[2 * i + 2] = dV[i + 1] * (t[2 * i + 1] - t[2 * i]) / 2;
            }
            chainGrad(n, i, false, data);
            data->gradCon[2 * i] = data->dR[n + 1].x * MAX_ACC / (MAX_VEL * MAX_VEL);
            data->gradCon[n + 2 * i] = data->dR[n + 1].y * MAX_ACC / (MAX_VEL * MAX_VEL);
            data->gradMin[2 * i] = data->dT[n + 2];
            data->gradCon[2 * n + 2 * i] = intGrad(n, data, i, false);
            data->gradMin[2 * i] += intGrad(n, data, i, true) * K_INTERSECT;
            // data->gradMin[2 * i] += (data->r[n + 1] - data->endPos) ^ data->dR[n + 1] * 2 / (MAX_VEL * MAX_VEL);
            data->gradCon[2 * n + 2 * i] *= (MAX_ACC / MAX_VEL);
            data->gradMin[2 * i] *= (MAX_ACC / MAX_VEL);
        }
        for (int i = 0; i < n / 2; i++)
        {
            dV[i] = dR[2 * i] = Point(0, 0);
            dT[2 * i] = 0;
            if (data->isLong[i])
            {
                shortFlag = true;
                dR[2 * i + 1] = 0;
                dR[2 * i + 2] = v[i + 1];
                dV[i + 1] = 0;
                dT[2 * i + 1] = 0;
                dT[2 * i + 2] = 1;
                for (int j = i + 1; j < n / 2; j++)
                {
                    dT[2 * j + 2] = dT[2 * j + 1] = dT[2 * i + 2];
                    dV[j + 1] = dV[j];
                    dR[2 * j + 1] = dR[2 * j + 2] = dR[2 * j];
                }
            }
            else
            {
                dT[2 * i + 1] = dT[2 * i + 2] = 1;
                shortFlag = false;
                dR[2 * i + 1] = dR[2 * i + 2] = v[i + 1];
                dV[i + 1] = a[i];
            }
            chainGrad(n, i, shortFlag, data);
            data->gradCon[2 * i + 1] = data->dR[n + 1].x * MAX_ACC / (MAX_VEL * MAX_VEL);
            data->gradCon[n + 2 * i + 1] = data->dR[n + 1].y * MAX_ACC / (MAX_VEL * MAX_VEL);
            data->gradMin[2 * i + 1] = data->dT[n + 2];
            data->gradCon[2 * n + 2 * i + 1] = intGrad(n, data, i, false);
            data->gradMin[2 * i + 1] += intGrad(n, data, i, true) * K_INTERSECT;
            // data->gradMin[2 * i + 1] += (data->r[n + 1] - data->endPos) ^ data->dR[n + 1] * 2 / (MAX_VEL * MAX_VEL);
            data->gradCon[2 * n + 2 * i + 1] *= (MAX_ACC / MAX_VEL);
            data->gradMin[2 * i + 1] *= (MAX_ACC / MAX_VEL);
        }
    }

    // Время проезда - то, что минимизируем
    double minimizing(unsigned n, const double *x, double *grad, void *voidData)
    {
        MetricsData *data = static_cast<MetricsData *>(voidData);
        mainActions(n, x, data);
        if (grad)
        {
            // for (int i = 0; i < int(n); i++)
            // {
            //     data->x[i] += 1e-6;
            //     countSections(n, data);
            //     grad[i] = ((countIntersections(n, data, true) * K_INTERSECT + data->t[n + 2]/* + (data->r[n + 1] - data->endPos).mag2() / (MAX_VEL * MAX_VEL)*/) * MAX_ACC / MAX_VEL - data->resultMin) / 1e-6;
            //     data->x[i] -= 1e-6;
            // }
            // for (int i = 0; i < int(n); i++)
            // {
            //     if (abs((grad[i] - data->gradMin[i]) / grad[i]) > 0.1) // && abs(grad[i]) > 0.02)
            //     {
            //         cout << "INCORRECT MIN " << grad[i] << " " << data->gradMin[i] << " " << i << endl;
            //         for (int j = 0; j < int(n); j++) {
            //             cout << x[j] << " ";
            //         }
            //         cout << endl;
            //     }
            //     else {
            //         cout << "ok" << endl;
            //     }
            // }
            for (int i = 0; i < int(n); i++)
            {
                grad[i] = data->gradMin[i];
            }
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
        // result[2] = data->resultCon[2];
        if (grad)
        {
            // for (int i = 0; i < int(n); i++)
            // {
            //     data->x[i] += 1e-8;
            //     countSections(n, data);
            //     grad[i] = ((data->r[n + 1] - data->endPos).x * MAX_ACC / (MAX_VEL * MAX_VEL) - result[0]) / 1e-6;
            //     grad[n + i] = ((data->r[n + 1] - data->endPos).y * MAX_ACC / (MAX_VEL * MAX_VEL) - result[1]) / 1e-6;
            //     data->x[i] -= 1e-6;
            // }
            // for (int i = 0; i < 2 * int(n); i++)
            // {
            //     if (abs((grad[i] - data->gradCon[i]) / grad[i]) > 0.02)
            //     {
            //         cout << "INCORRECT " << grad[i] << " " << data->gradCon[i] << " " << i << endl;
            //         // for (int j = 0; j < int(n); j++) {
            //         //     cout << x[j] << " ";
            //         // }
            //         // cout << endl;
            //     }
            // }
            for (int i = 0; i < 2 * int(n); i++)
            {
                grad[i] = data->gradCon[i];
            }
        }

        // Timer myTimer;
        // MetricsData drawData = *data;
        // drawer::clear();
        // drawer::drawWay(drawData);
        // myTimer.reset();
        // drawer::display();
        // while (myTimer.time() < 0.01)
        //     ;
    }
}