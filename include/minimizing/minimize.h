/*
великие алгоритмы минимизации
*/

#pragma once
#include <nlopt.h>
#include "../metrics/metrics.h"
#include "initialApprox.h"

void minimize(Point start, Point end, Point vStart, Point vEnd, AbsRigBody *enemies, int nEnemies)
{
    MetricsData data;
    data.pos = start;
    data.endPos = end;
    data.vel = vStart;
    data.endVel = vEnd;
    data.nEnemies = nEnemies;
    data.n = 2 * N_SOLVER_PAIRS;
    for (int i = 0; i < nEnemies; i++)
    {
        data.enemies[i] = enemies[i];
    }
    nlopt_opt opt = nlopt_create(NLOPT_LD_SLSQP, data.n); // создаем объект оптимайзера
    double lower_bounds[data.n], x[data.n];               // нижние границы для переменных - ноль для изменения по времени и минус бесконечность для ускорения
    for (int i = 0; i < data.n; i++)
    {
        if (i % 2)
            lower_bounds[i] = 0;
        else
            lower_bounds[i] = -1e14;
    }
    void *voidData = static_cast<void *>(&data);                 // переводим дату в войд для передачи в оптимайзер
    nlopt_set_lower_bounds(opt, lower_bounds);                   // собственно задаем нижние границы
    nlopt_set_min_objective(opt, metrics::minimizing, voidData); // задаем функцию для минимизации
    double tolerances[3] = {1e-4, 1e-4};
    nlopt_add_equality_mconstraint(opt, 2, metrics::constraints, voidData, tolerances); // задаем функцию ограничений
    nlopt_set_xtol_rel(opt, 1e-6);
    nlopt_set_ftol_rel(opt, 1e-6); // abs?? x?? - задаем критерий остановки - относительное изменение метрики меньше 1е-4 (мб сделать для иксов? или абсолютное?)

    initialApprox(start, end, vStart, vEnd, enemies, nEnemies, x, data.n);
    // x[0] = M_PI / 2;
    // x[1] = 0.5;

    // for (int i = 0; i < data.n; i++) {
    //     data.x[i] = x[i];
    //     cout << x[i] << endl;
    // }

    // drawer::drawWay(data);
    // drawer::display();
    double minf;
    // cout << "lol" << endl;
    // cout << "lalala" << endl;
    int result = nlopt_optimize(opt, x, &minf);
    std::cout << result << " " << minf << " " << data.resultCon[0] << " " << data.resultCon[1] << " " << data.resultCon[2] << "\n";
    nlopt_destroy(opt);
    drawer::drawWay(data);
}