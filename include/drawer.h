#pragma once

#include <SFML/Graphics.hpp>
#include "const.h"
#include "auxiliary/Point.h"
#include "metrics/MetricsData.h"
#include <iostream>

using namespace std;

namespace drawer
{
    double k_draw = min(double(PIXELS_X) / FIELD_DX, double(PIXELS_Y) / FIELD_DY);
    // Создаем окно размером 1000x800 пикселей.
    sf::RenderWindow window(sf::VideoMode(FIELD_DX *k_draw, FIELD_DY *k_draw), "apsdoopasd");

    // Определяем цвета для удобства.
    sf::Color colorField(34, 139, 34); // ForestGreen (Зеленый)
    sf::Color colorObs(220, 20, 60);   // Crimson (Красный)

    sf::Event event;

    void setFramerateLimit(unsigned int limit)
    {
        // Ограничиваем частоту кадров (FPS), чтобы не перегружать процессор.
        window.setFramerateLimit(limit);
    }

    int updateEvent()
    {
        int flag = 0;
        while (window.pollEvent(event))
        {
            // Если нажали "закрыть" (крестик), закрываем окно.
            if (event.type == sf::Event::Closed)
            {
                window.close();
                flag = 1;
            }
        }
        return flag;
    }

    void clear()
    {
        // Очищаем экран зеленым цветом поля.
        window.clear(colorField);
    }

    void display()
    {
        // Отображаем всё, что нарисовали, на экране.
        window.display();
    }

    void drawCircle(Point pos, double r, sf::Color col = sf::Color(0, 0, 0))
    {
        sf::CircleShape obstacle1(int(r * k_draw));                                                      // Радиус
        obstacle1.setFillColor(col);                                                                     // Красный цвет
        obstacle1.setPosition((FIELD_DX / 2 + pos.x - r) * k_draw, (FIELD_DY / 2 - pos.y - r) * k_draw); // координаты
        window.draw(obstacle1);                                                                          // Выводим на экран
    }

    void drawLine(Point p1, Point p2, double w, sf::Color col = sf::Color(0, 0, 0))
    {
        sf::RectangleShape line(sf::Vector2f((p2 - p1).mag() * k_draw, w * k_draw));
        line.rotate(-(p2 - p1).arg() * 180 / M_PI);
        line.setFillColor(col);
        line.setPosition((FIELD_DX / 2 + p1.x) * k_draw, (FIELD_DY / 2 - p1.y) * k_draw);
        window.draw(line);
    }
    void drawDumbBangBang(MetricsData data)
    {
        Point pos = data.pos, vel = data.vel, acc;
        double time;
        for (int i = 0; i < data.n/2; i++)
        {
            acc = Point(cos(data.x[i * 2]) * MAX_ACC, MAX_ACC * sin(data.x[i * 2]));
            time = data.x[i * 2 + 1];
            pos += vel * time + acc * time * time / 2;
            vel += acc * time;
            // std::cout<<(vel.mag()<MAX_VEL)<<"\n";
            drawer::drawCircle(pos, 50, sf::Color(128, 0, 255));
        }
    }
    void drawWay(MetricsData data, double w, double deltaT = 0.1)
    {
        double t = deltaT, tPlot;
        int i = 0;
        Point p = data.r[0], pNew;
        for (int j = 0; j < data.n; j++)
        {
            if (j % 2 || data.t[j + 1] >= 0)
            {
                drawCircle(data.r[j + 1], 50);
            }
        }
        while (t < data.t[data.n + 2])
        {
            while (data.t[i] < t)
            {
                i++;    
            }
            i--;
            if (data.t[i] < 0)
            {
                i--;
            }
            if (i % 2)
            {
                if (data.t[i] >= 0)
                {
                    tPlot = t - data.t[i];
                    pNew = data.r[i] + data.v[i / 2 + 1] * tPlot;
                    drawLine(p, pNew, w);
                    p = pNew;
                    cout << "say wallahi" << endl;
                }
            }
            else
            {
                tPlot = t - data.t[i];
                pNew = data.r[i] + data.v[i / 2] * tPlot + data.a[i / 2] * tPlot * tPlot / 2;
                drawLine(p, pNew, w);
                p = pNew;
            }
            t += deltaT;
        }
        pNew = data.r[data.n + 2];
        drawLine(p, pNew, w);
    }
}