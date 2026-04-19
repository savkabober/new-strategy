#include "include/auxiliary/numAux.h"
#include "include/Timer.h"
#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>
#include "include/const.h"
#include "include/drawer.h"
#include "include/metrics/metrics.h"

using namespace std;

int main()
{
    double prod[4], time;
    Timer timer;
    timer.reset();
    for (long i = 0; i < 1e6; i++) {
        numAux::solveEq(prod, 1, 2, 3, 4, 5);
    }
    time = timer.time();
    cout << time << endl;

    int nPairs = 4;

    double x[2 * nPairs], xData[2 * nPairs], tData[2 * nPairs + 4], result[3], grad[6 * nPairs],tMaxData[nPairs+1];
    Point rData[2 * nPairs + 4], vData[nPairs + 2], aData[nPairs + 1], uData[nPairs + 1];

    x[0] = M_PI / 2;
    x[1] = 1;
    x[2] = -M_PI / 4;
    x[3] = 1;
    x[4] = M_PI;
    x[5] = 1;
    x[6] = M_PI / 2;
    x[7] = 1;

    MetricsData data;

    data.x = xData;
    data.t = tData;
    data.r = rData;
    data.v = vData;
    data.a = aData;
    data.pos = Point(0, 0);
    data.vel = Point(0, 0);
    data.endPos = Point(1000, 0);
    data.endVel = Point(0, 0);
    data.enemies[0] = AbsRigBody(Point(0, 300), ROBOT_R, Point(0, 0));
    data.nEnemies = 1;
    data.n = nPairs * 2;
    data.tMax = tMaxData;
    
    void *data_ptr = static_cast<void *>(&data);

    metrics::constraints(result,2 * nPairs, x, grad, data_ptr);

    for (int i = 0; i <= nPairs * 2; i++)
    {
        if (i % 2 == 0 || data.t[i] >= 0) {
            cout << data.r[i].x << " " << data.r[i].y << endl;
        }
    }

    drawer::setFramerateLimit(60);

    drawer::clear();
    drawer::drawCircle(Point(0, 0), ROBOT_R);
    for (int i = 0; i < data.nEnemies; i++) {
        drawer::drawCircle(data.enemies[i].getPos(), ROBOT_R);
    }
    drawer::drawWay(data, 20);
    drawer::display();

    while (!drawer::updateEvent())
    {
    }
    return 0;
}
