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
    sf::RenderWindow window(sf::VideoMode(FIELD_DX *k_draw, FIELD_DY *k_draw), "megaknight67");

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

    void drawLine(Point p1, Point p2, double w = LINE_WIDTH, sf::Color col = sf::Color(0, 0, 0))
    {
        double ang = (p2 - p1).arg();
        sf::RectangleShape line(sf::Vector2f((p2 - p1).mag() * k_draw, w * k_draw));
        line.rotate(-ang * 180 / M_PI);
        line.setFillColor(col);
        line.setPosition((FIELD_DX / 2 + p1.x - w / 2 * sin(ang)) * k_draw, (FIELD_DY / 2 - p1.y - w / 2 * cos(ang)) * k_draw);
        window.draw(line);
    }
    void drawDumbBangBang(MetricsData data)
    {
        Point pos = data.pos, vel = data.vel, acc;
        double time;
        for (int i = 0; i < data.n / 2; i++)
        {
            acc = Point(cos(data.x[i * 2]) * MAX_ACC, MAX_ACC * sin(data.x[i * 2]));
            time = data.x[i * 2 + 1];
            pos += vel * time + acc * time * time / 2;
            vel += acc * time;
            drawer::drawCircle(pos, 50, sf::Color(128, 0, 255));
        }
    }
    void drawWay(MetricsData data, double w = LINE_WIDTH, double deltaT = 0.01)
    {
        drawer::drawCircle(data.pos, w+30, sf::Color(0, 0, 255));
        drawer::drawCircle(data.endPos, w+30, sf::Color(0, 0, 255));
        drawer::drawLine(data.pos, data.pos + data.vel, w, sf::Color(0, 0, 255));
        drawer::drawLine(data.endPos, data.endPos + data.endVel, w, sf::Color(0, 0, 255));
        double t = deltaT, tPlot;
        int i = 0;
        Point p = data.r[0], pNew;
        for (int j = 0; j < data.n / 2; j++)
        {
            drawCircle(data.r[2 * j + 1], w + 20, sf::Color(255, 128, 0));
            if (data.isLong[j])
            {
                drawCircle(data.r[2 * j + 2], w + 20, sf::Color(255, 128, 0));
            }
        }
        while (t < data.t[data.n + 2])
        {
            while (data.t[i] < t)
            {
                i++;
            }
            i--;
            if (i % 2)
            {
                if (data.isLong[i / 2]) {
                    tPlot = t - data.t[i];
                    pNew = data.r[i] + data.v[i / 2 + 1] * tPlot;
                    drawLine(p, pNew, w, sf::Color(255, 0, 0));
                    p = pNew;
                }
                
            }
            else
            {
                tPlot = t - data.t[i];
                pNew = data.r[i] + data.v[i / 2] * tPlot + data.a[i / 2] * tPlot * tPlot / 2;
                drawLine(p, pNew, w, sf::Color(0, 255, 128));
                p = pNew;
            }
            t += deltaT;
        }
        pNew = data.r[data.n + 2];
        drawLine(p, pNew, w);
    }

    void drawVel(Point p, Point v, double w = LINE_WIDTH) {
        drawLine(p, p + v * VELOCITY_K, w, sf::Color(128, 0, 255));
    }
}