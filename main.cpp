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
    int nEnemies = 6;
    AbsRigBody enemies[nEnemies];
    /*
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
    */
    enemies[0] = AbsRigBody(Point(150, -500), 2 * ROBOT_R, Point(0, 0));
    enemies[1] = AbsRigBody(Point(-600, 400), 2 * ROBOT_R, Point(0, 0));
    enemies[2] = AbsRigBody(Point(700, 300), 2 * ROBOT_R, Point(0, 0));
    enemies[3] = AbsRigBody(Point(401, 400), 4 * ROBOT_R, Point(0, 0));
    enemies[4] = AbsRigBody(Point(0, 1000), 2 * ROBOT_R, Point(0, 0));
    enemies[5] = AbsRigBody(Point(-250, 1300), 2 * ROBOT_R, Point(0, 0));

    drawer::setFramerateLimit(60);

    Timer myTimer;
    myTimer.reset();
    drawer::clear();

    for (int i = 0; i < nEnemies; i++)
    {
        drawer::drawCircle(enemies[i].getPos(), enemies[i].getRad() + 20, sf::Color(255, 0, 0));
        drawer::drawCircle(enemies[i].getPos(), enemies[i].getRad(), sf::Color(128, 255, 128));
    }
    // initialApprox(data.pos, data.endPos, data.vel, data.endVel, data.enemies, data.nEnemies, x, data.n);
    // for (int i = 0; i < data.n; i++) {
    //     cout << x[i] << endl;
    // }
    int nRepeat = 1;
    for (int i = 0; i < nRepeat; i++) {
        minimize(Point(0, 0), Point(0, 2000), Point(200, 500), Point(-500, 200), enemies, nEnemies);
    }
    

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

    // drawer::drawWay(data);
    // drawer::drawVel(data.endPos, data.endVel);
    // drawer::drawLine(Point(0, 0), Point(be));
    // drawer::drawLine(Point(0, 0), data.dR[2 * nPairs + 1]);
    drawer::display();
    long double deltaT = myTimer.time();
    cout << "time in ms: " << deltaT * 1000 / nRepeat << endl;
    while (!drawer::updateEvent())
    {
    }
    return 0;
}
