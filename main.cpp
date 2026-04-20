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
    int nPairs = 100;

    double xData[2 * nPairs], tData[2 * nPairs + 4], tMaxData[nPairs + 1];
    Point rData[2 * nPairs + 4], vData[nPairs + 2], aData[nPairs + 1];

    MetricsData data;

    data.x = xData;
    data.t = tData;
    data.r = rData;
    data.v = vData;
    data.a = aData;
    data.pos = Point(0, 0);
    data.vel = Point(1000, 1000);
    data.endPos = Point(1000, 1000);
    data.endVel = Point(1000, -1000);

    data.nEnemies = 0;
    data.n = nPairs * 2;
    data.tMax = tMaxData;

    for (int i = 0; i < nPairs * 2; i++)
        data.x[i] = 0;
    void *data_ptr = static_cast<void *>(&data);
    minimize(data);
    // metrics::countSections(data.n,data.x,data_ptr);
    for (int i = 0; i < nPairs; i++)
        std::cout << data.x[2 * i] << " " << data.x[2 * i + 1] << "\n";

    drawer::setFramerateLimit(60);
    drawer::clear();
    drawer::drawCircle(data.pos, 50, sf::Color(0, 0, 255));
    drawer::drawCircle(data.endPos, 50, sf::Color(0, 0, 255));
    drawer::drawLine(data.pos, data.pos + data.vel, 10, sf::Color(0, 0, 255));
    drawer::drawLine(data.endPos, data.endPos + data.endVel, 10, sf::Color(0, 0, 255));
    Point pos = data.pos, vel = data.vel, acc;
    double time;
    for (int i = 0; i < nPairs; i++)
    {
        acc = Point(cos(data.x[i * 2])*MAX_ACC,MAX_ACC*sin(data.x[i * 2]));
        time = data.x[i * 2 + 1];
        pos+=vel*time+acc*time*time/2;
        vel+=acc*time;
        // std::cout<<(vel.mag()<MAX_VEL)<<"\n";
        drawer::drawCircle(pos,50,sf::Color(128,0,255));
    }
    std::cout<<(pos-data.endPos).mag()<<"\n";
    // drawer::drawWay(data, 20);
    drawer::display();

    while (!drawer::updateEvent())
    {
    }
    return 1;
}
