#include "include/auxiliary/numAux.h"
#include "include/Timer.h"
#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>
#include "include/const.h"
#include "include/drawer.h"
#include "include/metrics/metrics.h"
#include "include/minimizing/minimize.h"
#include "include/minimizing/initialApprox.h"

using namespace std;
int main(void)
{
    /*
    Дата выглядит так
    Point pos, vel, endPos, endVel;
    int nEnemies, n;
    AbsRigBody enemies[MAX_ROBOT_COUNT];
    double *t, *x, *tMax, *gradCon, *gradMin, *tIntCon, *tIntMin, *resultCon, resultMin, *dT;
    bool *isLong;
    Point *a, *v, *r, *vMax, *dV, *dR, *dA;
    */
    int nPairs = 6, nEnemies = 6;
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
    data.endVel = Point(-500, 200);
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
    data.isLong = isLongData;
    data.a = aData;
    data.v = vData;
    data.r = rData;
    data.vMax = vMaxData;
    data.dV = dVData;
    data.dR = dRData;
    data.dA = dAData;
    data.enemies[0] = AbsRigBody(Point(200, 500), 2 * ROBOT_R, Point(0, 0));
    data.enemies[1] = AbsRigBody(Point(-600, 300), 2 * ROBOT_R, Point(0, 0));
    data.enemies[2] = AbsRigBody(Point(700, 300), 2 * ROBOT_R, Point(0, 0));
    data.enemies[3] = AbsRigBody(Point(400, 400), 2 * ROBOT_R, Point(0, 0));
    data.enemies[5] = AbsRigBody(Point(0, 500), 2 * ROBOT_R, Point(0, 0));
    data.enemies[4] = AbsRigBody(Point(-250, 300), 2 * ROBOT_R, Point(0, 0));
    // data.enemies[6] = AbsRigBody(Point(-1000, 1000), 2 * ROBOT_R, Point(0, 0));

    double resultCon[3], gradientCon[6 * nPairs], x[2 * nPairs], gradientMin[2 * nPairs], resultMin;
    // double be;
    // заполнение иксов рандомной датой для тестов
    // 6.21768 0.897666 2.19645 1.39516 0.532834 0.23632 -10.1611 0.0550037 1.9976 0.858517 -4.23334 1.07198e-14
    x[0] = 6.21768;
    x[1] = 0.897666;
    x[2] = 2.19645;
    x[3] = 1.39516;
    x[4] = 0.532834;
    x[5] = 0.23632;
    x[6] = -10.1611;
    x[7] = 0.0550037;
    x[8] = 1.9976;
    x[9] = 0.858517;
    x[10] = -4.23334;
    x[11] = 1.07198e-14;
    // x[12] = 0.313105;
    // x[13] = 0.0547674;
    // x[14] = -4.6205;
    // x[15] = 0.0541166;
    // x[16] = -4.37416;
    // x[17] = 0.258866;
    // x[18] = 184.77;
    // x[19] = 0.31216;
    void *voidData = static_cast<void *>(&data);

    // metrics::constraints(2, resultCon, 2 * nPairs, x, gradientCon, voidData);

    drawer::setFramerateLimit(60);

    Timer myTimer;
    myTimer.reset();
    drawer::clear();

    for (int i = 0; i < nEnemies; i++)
    {
        drawer::drawCircle(data.enemies[i].getPos(), 2 * ROBOT_R + 20, sf::Color(255, 0, 0));
        drawer::drawCircle(data.enemies[i].getPos(), 2 * ROBOT_R, sf::Color(128, 255, 128));
    }
    findWay(&data);
    // minimize(data);

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
