#include "include/auxiliary/numAux.h"
#include "include/Timer.h"
#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>
#include "include/const.h"
#include "include/drawer.h"
#include "include/metrics/metrics.h"
#include "include/minimizing/minimize.h"
using namespace std;
int main(void)
{
    /*
    Дата выглядит так
    Point pos, vel, endPos, endVel;
    int nEnemies, n;
    AbsRigBody enemies[MAX_ROBOT_COUNT];
    double *t, *x, *tMax, *gradCon, *gradMin, *tIntCon, *tIntMin, *resultCon, resultMin, safeDist;
    bool *isLong;
    Point *a, *v, *r, *vMax, *dV, *dR, *dA;
    */
    int nPairs = 10, nEnemies = 1;
    // Создание переменных для даты
    bool isLongData[nPairs + 1];
    double tData[2 * nPairs + 4], xData[2 * nPairs], tMaxData[nPairs], gradConData[6 * nPairs], gradMinData[2 * nPairs];
    double tIntConData[6 * (nPairs + 1) * nEnemies], tIntMinData[6 * (nPairs + 1) * nEnemies], resultConData[3];
    Point rData[2 * nPairs + 4], vData[nPairs + 2], aData[nPairs + 1], vMaxData[nPairs];
    Point dVData[nPairs + 2], dRData[nPairs + 1], dAData[nPairs + 1];
    // Заполнение даты, чтобы в ней все было
    // Если с кодом творится пиздец - смотри сюда!!! (все может крашится если ссылается на чето пустое)
    // В будущем стоит сделать все массивы с максимальным значением n. да, потратится сколько то памяти, но зато нет ебли с передачей
    MetricsData data;
    data.pos = Point(-1500, -1500);
    data.vel = Point(MAX_VEL, MAX_VEL);
    data.endPos = Point(1500, 1500);
    data.endVel = Point(MAX_VEL, 0);
    data.nEnemies = nEnemies;
    data.n = nPairs * 2;
    data.t = tData;
    data.x = xData;
    data.tMax = tMaxData;
    data.gradCon = gradConData;
    data.gradMin = gradMinData;
    data.tIntCon = tIntConData;
    data.tIntMin = tIntMinData;
    data.resultCon = resultConData;
    data.safeDist = 20;
    data.isLong = isLongData;
    data.a = aData;
    data.v = vData;
    data.r = rData;
    data.vMax = vMaxData;
    data.dV = dVData;
    data.dR = dRData;
    data.dA = dAData;
    data.enemies[0].setPos(Point(-400, 100));
    data.enemies[0].setVel(Point(500, 500));
    for (int i = 0; i < nEnemies; i++)
        data.enemies[i].setRad(ROBOT_R);

    // заполнение иксов рандомной датой для тестов
    //  data.x[0] = 0;
    //  data.x[1] = 1;
    //  data.x[2] = 1;
    //  data.x[3] = 0.5;
    //  data.x[4] = M_PI;
    //  data.x[5] = 1.5;
    //  data.x[6] = -M_PI / 2;
    //  data.x[7] = 3;

    drawer::setFramerateLimit(60);
    drawer::clear();
    for (int i = 0; i < nEnemies; i++)
    {
        drawer::drawCircle(data.enemies[i].getPos(), ROBOT_R,sf::Color(255,0,0));
        drawer::drawLine(data.enemies[i].getPos(), data.enemies[i].getPos() + data.enemies[i].getVel(), 10,sf::Color(255,0,0));
    }
    // for (int i = 0; i < 100; i++)
    // {
    //     data.vel = Point(MAX_VEL * cos(2 * M_PI / 100 * i), MAX_VEL * sin(2 * M_PI / 100 * i));
    //     for (int j = 0; j < 100; j++)
    //     {
    //         drawer::clear();
    //         data.endVel = Point(MAX_VEL * cos(2 * M_PI / 100 * j), MAX_VEL * sin(2 * M_PI / 100 * j));
    //         minimize(data);
    //         metrics::countSections(nPairs * 2, &data);
    //         drawer::drawWay(data, 20);
    //         drawer::display();
    //     }
    // }
    minimize(data);
    metrics::countSections(nPairs * 2, &data);
    drawer::drawWay(data, 20);
    drawer::display();
    while (!drawer::updateEvent())
    {
    }
    return 0;
}
