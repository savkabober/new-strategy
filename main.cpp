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
    int nPairs = 6, nEnemies = 3;
    // Создание переменных для даты
    bool isLongData[nPairs + 1];
    double tData[2 * nPairs + 4], xData[2 * nPairs], tMaxData[nPairs], gradConData[6 * nPairs], gradMinData[2 * nPairs];
    double tIntConData[6 * (nPairs + 1) * nEnemies], tIntMinData[6 * (nPairs + 1) * nEnemies], resultConData[3], dTData[2 * nPairs + 4];
    Point rData[2 * nPairs + 4], vData[nPairs + 2], aData[nPairs + 1], vMaxData[nPairs];
    Point dVData[nPairs + 2], dRData[2 * nPairs + 4], dAData[nPairs + 1];
    // Заполнение даты, чтобы в ней все было
    // Если с кодом творится пиздец - смотри сюда!!! (все может крашится если ссылается на чето пустое)
    // В будущем стоит сделать все массивы с максимальным значением n. да, потратится сколько то памяти, но зато нет ебли с передачей
    MetricsData data;
    data.pos = Point(0, 0);
    data.vel = Point(200, 500);
    data.endPos = Point(0, 2000);
    data.endVel = Point(-200, 200);
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
    data.dT = dTData;
    data.safeDist = 20;
    data.isLong = isLongData;
    data.a = aData;
    data.v = vData;
    data.r = rData;
    data.vMax = vMaxData;
    data.dV = dVData;
    data.dR = dRData;
    data.dA = dAData;
    data.enemies[0] = AbsRigBody(Point(100, 500), 2 * ROBOT_R, Point(0, 0));
    data.enemies[1] = AbsRigBody(Point(0, 1100), 2 * ROBOT_R, Point(0, 0));
    data.enemies[2] = AbsRigBody(Point(-200, 750), 2 * ROBOT_R, Point(0, 0));
    data.enemies[3] = AbsRigBody(Point(-1003, 1000), 2 * ROBOT_R, Point(0, 0));
    data.enemies[4] = AbsRigBody(Point(-1004, 1000), 2 * ROBOT_R, Point(0, 0));
    data.enemies[5] = AbsRigBody(Point(-1005, 1000), 2 * ROBOT_R, Point(0, 0));
    // data.enemies[6] = AbsRigBody(Point(-1000, 1000), 2 * ROBOT_R, Point(0, 0));

    double resultCon[3], gradientCon[6 * nPairs], x[2 * nPairs], gradientMin[2 * nPairs], resultMin;
    // double be;
    // заполнение иксов рандомной датой для тестов
    // 2.30035 0.1 1.68888 0.422166 1.3909 0.410584 1.22203 0.460373 1.58416 0.561293 -2.1624 0.1 -2.22767 0.1 -2.0797 0.1 -2.09383 0.1 -0.492633 0.1
    x[0] = 2.30035;
    x[1] = 0.1;
    x[2] = 1.68888;
    x[3] = 0.422166;
    x[4] = 1.3909;
    x[5] = 0.410584;
    x[6] = 1.22203;
    x[7] = 0.460373;
    x[8] = 1.58416;
    x[9] = 0.561293;
    x[10] = -2.1624;
    x[11] = 0.1;
    x[12] = -2.22767;
    x[13] = 0.1;
    x[14] = -2.0797;
    x[15] = 0.1;
    x[16] = -2.09383;
    x[17] = 0.1;
    x[18] = -0.492633;
    x[19] = 0.1;
    void *voidData = static_cast<void *>(&data);

    // metrics::constraints(2, resultCon, 2 * nPairs, x, gradientCon, voidData);

    drawer::setFramerateLimit(60);

    Timer myTimer;
    myTimer.reset();
    drawer::clear();
    minimize(data);

    // metrics::constraints(2, resultCon, 2 * nPairs, x, gradientCon, voidData);
    // resultMin = metrics::minimizing(2 * nPairs, x, gradientMin, voidData);

    // x[6] += 1e-8;
    // double be = resultMin; // data.tIntMin[94] - data.tIntMin[90];
    // metrics::constraints(2, resultCon, 2 * nPairs, x, gradientCon, voidData);
    // resultMin = metrics::minimizing(2 * nPairs, x, gradientMin, voidData);
    // be = (resultMin - be) / 1e-8; // (data.tIntMin[94] - data.tIntMin[90] - be) / 1e-6;
    // cout << be << " " << gradientMin[6] << endl;
    // cout << data.t[9] - data.t[8] + data.tIntMin[94] - data.tIntMin[90] << endl;
    // for (int i = 0; i < 6 * (nPairs + 1) * nEnemies; i++)
    // {
    //     // cout << data.tIntMin[i] << " " << i << endl;
    //     if (data.tIntMin[i] >= 0)
    //     {
    //         int z = i % (6 * (nPairs + 1)) / 6;
    //         if (i % (6 * (nPairs + 1)) % 6 < 4)
    //         {
    //             drawer::drawCircle(data.r[2 * z] + data.v[z] * data.tIntMin[i] + data.a[z] * data.tIntMin[i] * data.tIntMin[i] / 2, 50);
    //         }
    //         else
    //         {
    //             drawer::drawCircle(data.r[2 * z + 1] + data.v[z + 1] * data.tIntMin[i], 50);
    //         }
    //     }
    // }
    
    for (int i = 0; i < nEnemies; i++)
    {
        drawer::drawCircle(data.enemies[i].getPos(), 2 * ROBOT_R + 20, sf::Color(255, 0, 0));
        drawer::drawCircle(data.enemies[i].getPos(), 2 * ROBOT_R, sf::Color(128, 255, 128));
    }
    drawer::drawWay(data);
    drawer::drawVel(data.endPos, data.endVel);
    // drawer::drawLine(Point(0, 0), Point(be));
    // drawer::drawLine(Point(0, 0), data.dR[2 * nPairs + 1]);
    drawer::display();
    long double deltaT = myTimer.time();
    cout << "time in ms: " << deltaT * 1000 << endl;
    while (!drawer::updateEvent())
    {
    }
    return 0;
}
