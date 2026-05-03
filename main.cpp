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
    int nRepeat = 1;
    for (int i = 0; i < nRepeat; i++) {
        minimize(Point(0, 0), Point(0, 2000), Point(200, 500), Point(-500, 200), enemies, nEnemies);
    }

    drawer::display();
    long double deltaT = myTimer.time();
    cout << "time in ms: " << deltaT * 1000 / nRepeat << endl;
    while (!drawer::updateEvent())
    {
    }
    return 0;
}
