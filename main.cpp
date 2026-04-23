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
    Point dVData[nPairs + 2], dRData[2 * nPairs + 2], dAData[nPairs + 1];
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
    data.enemies[0] = AbsRigBody(Point(900, 500), 2 * ROBOT_R, Point(0, 0));
    data.enemies[1] = AbsRigBody(Point(-1001, 1000), 2 * ROBOT_R, Point(0, 0));
    data.enemies[2] = AbsRigBody(Point(-1002, 1000), 2 * ROBOT_R, Point(0, 0));
    data.enemies[3] = AbsRigBody(Point(-1003, 1000), 2 * ROBOT_R, Point(0, 0));
    data.enemies[4] = AbsRigBody(Point(-1004, 1000), 2 * ROBOT_R, Point(0, 0));
    data.enemies[5] = AbsRigBody(Point(-1005, 1000), 2 * ROBOT_R, Point(0, 0));
    // data.enemies[6] = AbsRigBody(Point(-1000, 1000), 2 * ROBOT_R, Point(0, 0));

    double resultCon[3], gradientCon[6 * nPairs], x[2 * nPairs], gradientMin[2 * nPairs], resultMin;
    double be;
    // заполнение иксов рандомной датой для тестов
    x[0] = 0;
    x[1] = 1;
    x[2] = 1;
    x[3] = 0.5;
    x[4] = M_PI;
    x[5] = 1.5;
    x[6] = -M_PI / 2;
    x[7] = 2;
    x[8] = 0;
    x[9] = 0.5;
    x[10] = M_PI / 2;
    x[11] = 1.2;
    x[12] = -1;
    x[13] = 0.75;
    x[14] = 3 * M_PI / 4;
    x[15] = 1.5;
    x[16] = 0;
    x[17] = 1;
    x[18] = -M_PI / 4;
    x[19] = 1.75;
    void *voidData = static_cast<void *>(&data);

    Timer myTimer;
    myTimer.reset();
    for (int i = 0; i < 1e4; i++) {
        metrics::constraints(3, resultCon, 2 * nPairs, x, gradientCon, voidData);
    }
    //resultMin = metrics::minimizing(2 * nPairs, x, gradientMin, voidData);
    long double deltaT = myTimer.time();
    cout << "time in mcs: " << deltaT * 1e6 / 1e4 << endl;

    minimize(data);

    // minimize(data);
    metrics::countSections(data.n,&data);

    drawer::setFramerateLimit(60);
    drawer::clear();
    for (int i = 0; i < nEnemies; i++)
    {
        drawer::drawCircle(data.enemies[i].getPos(), 2 * ROBOT_R,sf::Color(128,255,128));
        // drawer::drawLine(data.enemies[i].getPos(), data.enemies[i].getPos() + data.enemies[i].getVel(), 10,sf::Color(255,0,0));
    }

    // drawer::drawDumbBangBang(data);
    drawer::drawWay(data);
    // drawer::drawVel(data.endPos, data.endVel);
    drawer::drawVel(data.endPos, data.v[nPairs + 1]);
    drawer::display();
    while (!drawer::updateEvent())
    {
    }
    return 0;
}
