/*
начальное приближение бенг бенгами
*/

#pragma once
#include "bangBang.h"
#include "../metrics/metrics.h"
#include "../drawer.h"

void deleteRbt(int n, AbsRigBody *enemies, int i)
{
    for (int j = i; j < n - 1; j++)
    {
        enemies[j] = enemies[j + 1];
    }
}

bool findTangents(Point start, Point end, AbsRigBody *groups, int nEnemies, int *idxGroups, int nGroups, Point &leftTg, Point &rightTg)
{
    // Сначала разобъем всех роботов на группы
    Point prod[2], leftRbt[nEnemies], rightRbt[nEnemies];
    double angle1, angle2;
    int i, k;
    bool isInt[nGroups], isNew, flag;
    Point leftGr[nGroups], rightGr[nGroups];
    // теперь смотрим на пересечения с прямой
    for (i = 0; i < nGroups; i++)
    {
        isInt[i] = false;
        for (int j = idxGroups[i]; j < idxGroups[i + 1]; j++)
        {
            // cout << vecAux::lineCircleIntersect(prod, data->pos, data->endPos, groups[j].getPos(), groups[j].getRad()) << endl;
            if (vecAux::lineCircleIntersect(prod, start, end, groups[j].getPos(), groups[j].getRad()))
                isInt[i] = true;
        }
    }
    flag = false;
    for (i = 0; i < nGroups; i++) {
        if (isInt[i])
            flag = true;
    }
    if (!flag)
        return false;
    // теперь определим максимальные углы у каждого робота
    for (i = 0; i < nGroups; i++)
    {
        if (!isInt[i])
            continue;
        for (int j = idxGroups[i]; j < idxGroups[i + 1]; j++)
        {
            if ((groups[j].getPos() - start).mag() < groups[j].getRad() + SAFE_DIST)
            {
                isInt[i] = false;
                break;
            }
            vecAux::getTangentPoints(prod, start, groups[j].getPos(), groups[j].getRad());
            if (vecAux::getAngleBetweenPoints(prod[0], start, groups[j].getPos()) > 0)
            {
                leftRbt[j] = prod[0];
                rightRbt[j] = prod[1];
            }
            else
            {
                leftRbt[j] = prod[1];
                rightRbt[j] = prod[0];
            }
        }
    }
    // теперь определяем максимальный угол в группе
    for (i = 0; i < nGroups; i++)
    {
        if (!isInt[i])
            continue;
        k = idxGroups[i];
        do
        {
            isNew = false;
            for (int j = idxGroups[i]; j < idxGroups[i + 1]; j++)
            {
                if (k == j)
                    continue;
                if (vecAux::getAngleBetweenPoints(leftRbt[j], start, leftRbt[k]) > 0 && vecAux::getAngleBetweenPoints(leftRbt[k], start, rightRbt[j]) > 0)
                {
                    isNew = true;
                    k = j;
                    break;
                }
            }
        } while (k != idxGroups[i] && isNew);
        if (k == idxGroups[i] && isNew)
        {
            isInt[i] = false;
            continue;
        }
        leftGr[i] = leftRbt[k];
        k = idxGroups[i];
        do
        {
            isNew = false;
            for (int j = idxGroups[i]; j < idxGroups[i + 1]; j++)
            {
                if (k == j)
                    continue;
                if (vecAux::getAngleBetweenPoints(leftRbt[j], start, rightRbt[k]) > 0 && vecAux::getAngleBetweenPoints(rightRbt[k], start, rightRbt[j]) > 0)
                {
                    isNew = true;
                    k = j;
                    break;
                }
            }
        } while (k != idxGroups[i] && isNew);
        if (k == idxGroups[i] && isNew)
        {
            isInt[i] = false;
            continue;
        }
        rightGr[i] = rightRbt[k];
    }
    // а теперь вернем две лучшие касательные с обеих сторон
    for (i = 0; i < nGroups; i++) {
        if (isInt[i]) {
            leftTg = leftGr[i];
            rightTg = rightGr[i];
        }
    }
    for (i = 0; i < nGroups; i++) {
        if (isInt[i]) {
            leftTg = leftGr[i];
            rightTg = rightGr[i];
            i++;
            break;
        }
    }
    for (; i < nGroups; i++) {
        if (isInt[i]) {
            angle1 = vecAux::getAngleBetweenPoints(leftGr[i], start, end);
            if (angle1 < 0) {
                angle1 += 2 * M_PI;
            }
            angle2 = vecAux::getAngleBetweenPoints(leftTg, start, end);
            if (angle2 < 0) {
                angle2 += 2 * M_PI;
            }
            if (angle1 > angle2) {
                leftTg = leftGr[i];
            }
            angle1 = vecAux::getAngleBetweenPoints(end, start, rightGr[i]);
            if (angle1 < 0) {
                angle1 += 2 * M_PI;
            }
            angle2 = vecAux::getAngleBetweenPoints(end, start, rightTg);
            if (angle2 < 0) {
                angle2 += 2 * M_PI;
            }
            if (angle1 > angle2) {
                rightTg = rightGr[i];
            }
        }
    }
    return true;
}

int findGroups(AbsRigBody *groups, int *idxGroups, AbsRigBody *enemies, int nEnemies) {
    // Сначала разобъем всех роботов на группы
    AbsRigBody enemies2[nEnemies];
    for (int i = 0; i < nEnemies; i++)
    {
        enemies2[i] = enemies[i];
        groups[i] = enemies[i];
    }
    int nGroups = 0, k = 0;
    idxGroups[0] = 0;
    for (int i = 0; i < nEnemies;)
    {
        groups[i] = enemies2[0];
        deleteRbt(nEnemies - k, enemies2, 0);
        k++;
        for (; i < k; i++)
        {
            for (int j = 0; j < nEnemies - k; j++)
            {
                if ((enemies2[j].getPos() - groups[i].getPos()).mag2() < numAux::square(enemies2[j].getRad() + groups[i].getRad()))
                {
                    groups[k] = enemies2[j];
                    deleteRbt(nEnemies - k, enemies2, j);
                    k++;
                    j--;
                }
            }
        }
        nGroups++;
        idxGroups[nGroups] = i;
    }
    int nGroups;
}

void initialApprox(Point start, Point end, AbsRigBody *enemies, int nEnemies)
{
    int idxGroups[nEnemies + 1], nGroups;
    AbsRigBody groups[nEnemies];
    nGroups = findGroups(groups, enemies, nEnemies);
}