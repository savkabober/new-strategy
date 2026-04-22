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
    int nPairs = 4, nEnemies = 0;
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
    data.pos = Point(0, 0);
    data.vel = Point(0, 0);
    data.endPos = Point(2000, 0);
    data.endVel = Point(0, 0);
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

    //заполнение иксов рандомной датой для тестов
    data.x[0] = 0;
    data.x[1] = 1;
    data.x[2] = 1;
    data.x[3] = 0.5;
    data.x[4] = M_PI;
    data.x[5] = 1.5;
    data.x[6] = -M_PI / 2;
    data.x[7] = 3;

    metrics::countSections(nPairs * 2, &data);

    drawer::setFramerateLimit(60);
    drawer::clear();
    drawer::drawCircle(data.pos, 50, sf::Color(0, 0, 255));
    drawer::drawCircle(data.endPos, 50, sf::Color(0, 0, 255));
    drawer::drawLine(data.pos, data.pos + data.vel, 10, sf::Color(0, 0, 255));
    drawer::drawLine(data.endPos, data.endPos + data.endVel, 10, sf::Color(0, 0, 255));
    // for(int i = 0;i<2*nPairs+4;i++)
    // {
    //     drawer::drawCircle(data.r[i],10);
    // }

    // drawer::drawDumbBangBang(data);
    //drawer::drawWay(data, 20);
    drawer::display();

    while (!drawer::updateEvent())
    {
    }
    return 1;
}
