/*
начальное приближение бенг бенгами
*/

#pragma once
#include "bangBang.h"
#include "../metrics/metrics.h"
#include "../drawer.h"
#include <vector>

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
            vecAux::getTangentPoints(prod, start, groups[j].getPos(), groups[j].getRad() + SAFE_DIST / 2);
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
    for (i = 0; i < nGroups; i++)
    {
        if (isInt[i])
        {
            leftTg = leftGr[i];
            rightTg = rightGr[i];
        }
    }
    for (i = 0; i < nGroups; i++)
    {
        if (isInt[i])
        {
            leftTg = leftGr[i];
            rightTg = rightGr[i];
            i++;
            break;
        }
    }
    for (; i < nGroups; i++)
    {
        if (isInt[i])
        {
            angle1 = vecAux::getAngleBetweenPoints(leftGr[i], start, end);
            if (angle1 < 0)
            {
                angle1 += 2 * M_PI;
            }
            angle2 = vecAux::getAngleBetweenPoints(leftTg, start, end);
            if (angle2 < 0)
            {
                angle2 += 2 * M_PI;
            }
            if (angle1 > angle2)
            {
                leftTg = leftGr[i];
            }
            angle1 = vecAux::getAngleBetweenPoints(end, start, rightGr[i]);
            if (angle1 < 0)
            {
                angle1 += 2 * M_PI;
            }
            angle2 = vecAux::getAngleBetweenPoints(end, start, rightTg);
            if (angle2 < 0)
            {
                angle2 += 2 * M_PI;
            }
            if (angle1 > angle2)
            {
                rightTg = rightGr[i];
            }
        }
    }
    flag = false;
    for (i = 0; i < nGroups; i++)
    {
        if (isInt[i])
            flag = true;
    }
    if (!flag)
        return false;
    return true;
}

int findGroups(AbsRigBody *groups, int *idxGroups, AbsRigBody *enemies, int nEnemies)
{
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
    return nGroups;
}

int findWay(Point start, Point end, AbsRigBody *enemies, int nEnemies, Point *way, int nPoints)
{
    Point leftTg, rightTg;
    int idxGroups[nEnemies + 1], nGroups, nSteps = 100, length = 2;
    double newDist;
    AbsRigBody groups[nEnemies];
    nGroups = findGroups(groups, idxGroups, enemies, nEnemies);
    std::vector<Point> points;
    std::vector<int> parents;
    std::vector<int> lengths;
    std::vector<double> distances;
    points.push_back(start);
    parents.push_back(-1);
    distances.push_back(0);
    lengths.push_back(0);
    points.push_back(end);
    parents.push_back(0);
    distances.push_back(-1);
    lengths.push_back(1);
    for (int i = 1; i < nSteps; i++)
    {
        if (i >= length)
            break;
        if (findTangents(points[parents[i]], points[i], groups, nEnemies, idxGroups, nGroups, leftTg, rightTg))
        {
            points.push_back(leftTg);
            parents.push_back(parents[i]);
            distances.push_back((points[parents[i]] - leftTg).mag() + distances[parents[i]]);
            lengths.push_back(lengths[parents[i]] + 1);
            points.push_back(rightTg);
            parents.push_back(parents[i]);
            distances.push_back((points[parents[i]] - rightTg).mag() + distances[parents[i]]);
            lengths.push_back(lengths[parents[i]] + 1);
            length += 2;
        }
        else
        {
            if (findTangents(points[i], points[1], groups, nEnemies, idxGroups, nGroups, leftTg, rightTg) && lengths[i] < nPoints - 1)
            {
                points.push_back(leftTg);
                parents.push_back(i);
                distances.push_back((leftTg - points[i]).mag() + distances[i]);
                lengths.push_back(lengths[i] + 1);
                points.push_back(rightTg);
                parents.push_back(i);
                distances.push_back((rightTg - points[i]).mag() + distances[i]);
                lengths.push_back(lengths[i] + 1);
                length += 2;
            }
            else
            {
                if (distances[1] < 0)
                {
                    distances[1] = (points[1] - points[i]).mag() + distances[i];
                    if (i != 1)
                    {
                        parents[1] = i;
                        lengths[1] = lengths[i] + 1;
                    }
                    nSteps = i + 1 + 10;
                }
                else
                {
                    newDist = (points[1] - points[i]).mag() + distances[i];
                    if (newDist < distances[1])
                    {
                        distances[1] = newDist;
                        parents[1] = i;
                        lengths[1] = lengths[i] + 1;
                    }
                }
            }
        }
    }
    // cout << "lalala " << lengths[1] << endl;
    int nowPoint = 1;
    for (int i = lengths[1]; i >= 0; i--)
    {
        // cout << nowPoint << " " << parents[nowPoint] << endl;
        way[i] = points[nowPoint];
        nowPoint = parents[nowPoint];
    }
    // cout << "bebebe" << endl;
    return lengths[1] + 1;
}

void initialApprox(Point start, Point end, Point vStart, Point vEnd, AbsRigBody *enemies, int nEnemies, double *x, int n)
{
    int nPoints = n / 4, length, repeats, j = 0, plus, remainder;
    double accTime, constTime, decTime, cA, l, angle;
    Point way[nPoints + 1], vels[nPoints + 1];
    length = findWay(start, end, enemies, nEnemies, way, nPoints);
    for (int i = 1; i < length - 1; i++)
    {
        vels[i] = (way[i] - way[i - 1]).unity() * MAX_VEL;
    }
    vels[0] = vStart;
    vels[length - 1] = vEnd;
    repeats = n / 4 / (length - 1);
    remainder = n / 2 - repeats * (length - 1) * 2;
    for (int i = 0; i < length - 1; i++)
    {
        Point Vm = bangBang::bangBang(vels[i], vels[i + 1], way[i + 1] - way[i], MAX_ACC, MAX_VEL);
        // cout << vels[i] << " " << vels[i + 1] << " " << way[i + 1] << " " << way[i] << endl;
        accTime = (vels[i] - Vm).mag() / MAX_ACC;
        decTime = (vels[i + 1] - Vm).mag() / MAX_ACC;
        constTime = ((way[i + 1] - way[i]) - (vels[i] + Vm) / 2 * accTime - (vels[i + 1] + Vm) / 2 * decTime).mag() / MAX_VEL;
        // cout << accTime << " " << constTime << " " << accTime << " " << MAX_VEL - Vm.mag() << endl;
        if (MAX_VEL - Vm.mag() > EPSILON)
        {
            cA = cos(vecAux::getAngleBetweenPoints(vels[i], Point(0, 0), (Vm - vels[i])));
            l = -cA * vels[i].mag() + sqrt(cA * cA * vels[i].mag2() - (vels[i].mag2() - MAX_VEL * MAX_VEL));
            angle = (vels[i] + (Vm - vels[i]).unity() * l).arg();
        }
        else
        {
            angle = Vm.arg();
        }
        if (i * 2 < remainder)
            plus = repeats + 1;
        else
            plus = repeats;
        for (int k = j; k < j + plus; k++)
        {
            x[2 * k] = angle;
            x[2 * k + 1] = (accTime + constTime) / plus;
            // cout << x[2 * k + 1] << " " << plus << endl;
        }
        j += plus;
        cA = cos(vecAux::getAngleBetweenPoints(Vm, Point(0, 0), (vels[i + 1] - Vm)));
        l = -cA * Vm.mag() + sqrt(cA * cA * Vm.mag2() - (Vm.mag2() - MAX_VEL * MAX_VEL));
        if (i * 2 + 1 < remainder)
            plus = repeats + 1;
        else
            plus = repeats;
        for (int k = j; k < j + plus; k++)
        {
            x[2 * k] = (Vm + (vels[i + 1] - Vm).unity() * l).arg();
            x[2 * k + 1] = decTime / plus;
        }
        j += plus;
    }
    x[n - 1] /= 2;
}