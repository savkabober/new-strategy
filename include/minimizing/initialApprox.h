/*
начальное приближение бенг бенгами
*/

#pragma once
#include "bangBang.h"
#include "../metrics/metrics.h"

void deleteRbt(int n, AbsRigBody *enemies, int i)
{
    for (int j = i, j < n - 1; j++)
    {
        enemies[j] = enemies[j + 1];
    }
}

bool initialApprox(MetricsData *data)
{
    // Сначала разобъем всех роботов на группы
    AbsRigBody enemies2[data->nEnemies], groups[data->nEnemies];
    Point prod[2], leftRbt[data->nEnemies], rightRbt[data->nEnemies];
    for (int i = 0; i < data->nEnemies; i++)
    {
        enemies2[i] = data->enemies[i];
        groups[i] = data->nEnemies[i];
    }
    int nGroups = 0, k = 0, idxGroups[data->nEnemies + 1], nProd;
    bool isNew;
    idxGroups[0] = 0;
    for (int i = 0; i < data->nEnemies;)
    {
        groups[i] = enemies2[0];
        deleteRbt(data->nEnemies - k, enemies2, 0);
        k++;
        for (; i < k; i++)
        {
            for (int j = 0; j < data->nEnemies - k; j++)
            {
                if ((enemies2[j].getPos() - groups[i].getPos()).mag2() < numAux::square(enemies2[j].getRad() + groups[i].getRad()))
                {
                    groups[k] = enemies2[j];
                    deleteRbt(data->nEnemies - k, enemies2, j);
                    k++;
                    j--;
                }
            }
        }
        nGroups++;
        idxGroups[nGroups] = i;
    }
    bool isIntGr[nGroups], isIntRbt[data->nEnemies];
    Point leftGr[nGroups], rightGr[nGroups];
    // теперь смотрим на пересечения с прямой
    for (int i = 0; i < nGroups; i++)
    {
        isIntGr[i] = false;
        for (int j = idxGroups[i]; j < idxGroups[i + 1]; j++)
        {
            if (vecAux::lineCircleIntersect(prod, data->pos, data->endPos, groups[j].getPos(), groups[j].getRad()))
                isIntRbt[j] = isIntGr[i] = true;
            else
                isIntRbt[j] = false;
        }
    }
    // теперь определим максимальные углы у каждого робота
    for (int i = 0; i < nGroups; i++)
    {
        if (!isIntGr[i])
            break;
        for (int j = idxGroups[i]; j < idxGroups[i + 1]; j++)
        {
            nProd = vecAux::getTangentPoints(prod, data->pos, groups[i].getPos(), groups[i].getRad());
            if (nProd < 2)
                break;
            if (vecAux::getAngleBetweenPoints(prod[0], data->pos, groups[i].getPos()) > 0)
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
    for (int i = 0; i < nGroups; i++)
    {
        k = idxGroups[i];
        do
        {
            isNew = false;
            for (int j = idxGroups[i]; j < idxGroups[i + 1]; j++)
            {
                if (k == j)
                    continue;
                if (vecAux::getAngleBetweenPoints(leftRbt[j], data->pos, leftRbt[k]) > 0 && vecAux::getAngleBetweenPoints(leftRbt[k], data->pos, rightRbt[j]) > 0)
                {
                    isNew = true;
                    k = j;
                    break;
                }
            }
        } while (k != idxGroups[i] && isNew);
        if (k == idxGroups[i])
            return false;
        else
            leftGr[i] = leftRbt[k];
        k = idxGroups[i];
        do
        {
            isNew = false;
            for (int j = idxGroups[i]; j < idxGroups[i + 1]; j++)
            {
                if (k == j)
                    continue;
                if (vecAux::getAngleBetweenPoints(leftRbt[j], data->pos, rightRbt[k]) > 0 && vecAux::getAngleBetweenPoints(rightRbt[k], data->pos, rightRbt[j]) > 0)
                {
                    isNew = true;
                    k = j;
                    break;
                }
            }
        } while (k != idxGroups[i] && isNew);
        if (k == idxGroups[i])
            return false;
        else
            rightGr[i] = rightRbt[k];
    }
}