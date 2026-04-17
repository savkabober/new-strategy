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
    // double prod[4], time;
    // Timer timer;
    // timer.reset();
    // for (long i = 0; i < 1e6; i++) {
    //     numAux::solveEq(prod, 1, 2, 3, 4, 5);
    // }
    // time = timer.time();
    // cout << time << endl;

    int nPairs = 4;

    double x[2 * nPairs], xData[2 * nPairs], tData[2 * nPairs + 4], result[3], grad[6 * nPairs];
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
    data.u = uData;
    data.pos = Point(0, 0);
    data.vel = Point(0, 0);
    data.endPos = Point(1000, 0);
    data.endVel = Point(0, 0);
    data.enemies[0] = AbsRigBody(Point(0, 300), ROBOT_R, Point(0, 0));
    data.nEnemies = 1;
    data.n = nPairs * 2;

    void *data_ptr = static_cast<void *>(&data);

    metrics::constraints(3, result, 2 * nPairs, x, grad, data_ptr);

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

// int main() {
//     // 1. Создаем окно размером 1000x800 пикселей.
//     sf::RenderWindow window(sf::VideoMode(1000, 800), "Робот на зеленом поле");

//     // Ограничиваем частоту кадров (FPS), чтобы не перегружать процессор.
//     window.setFramerateLimit(60);

//     // 2. Определяем цвета для удобства.
//     sf::Color colorField(34, 139, 34); // ForestGreen (Зеленый)
//     sf::Color colorObs(220, 20, 60);     // Crimson (Красный)

//     // 3. Главный цикл окна. Он работает, пока окно открыто.
//     while (window.isOpen()) {
//         // --- Обработка событий ---
//         sf::Event event;
//         // Проверяем, произошли ли какие-то события (нажатие кнопок, мышки и т.д.)
//         while (window.pollEvent(event)) {
//             // Если нажали "закрыть" (крестик), закрываем окно.
//             if (event.type == sf::Event::Closed)
//                 window.close();
//         }

//         // --- Отрисовка ---

//         // 4. Очищаем экран зеленым цветом поля.
//         window.clear(colorField);

//         // 5. РИСУЕМ КРУЖКИ (Препятствия)

//         // Кружок 1 (Маленький, в центре)
//         sf::CircleShape obstacle1(40.f);     // Радиус 40
//         obstacle1.setFillColor(colorObs);   // Красный цвет
//         obstacle1.setPosition(500, 400);    // Координаты X, Y (верхний левый угол)
//         window.draw(obstacle1);             // Выводим на экран

//         // Кружок 2 (Большой, слева вверху)
//         sf::CircleShape obstacle2(70.f);     // Радиус 70
//         obstacle2.setFillColor(colorObs);   // Красный цвет
//         obstacle2.setPosition(150, 150);    // Координаты X, Y
//         window.draw(obstacle2);             // Выводим на экран

//         // Кружок 3 (Средний, справа внизу)
//         sf::CircleShape obstacle3(55.f);     // Радиус 55
//         obstacle3.setFillColor(colorObs);   // Красный цвет
//         obstacle3.setPosition(750, 600);    // Координаты X, Y
//         window.draw(obstacle3);             // Выводим на экран

//         // 6. Отображаем всё, что нарисовали, на экране.
//         window.display();
//     }

//     return 0;
// }