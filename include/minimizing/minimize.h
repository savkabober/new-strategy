/*
великие алгоритмы минимизации
*/

#pragma once
#include <nlopt.h>
#include "../metrics/metrics.h"
#include "initialApprox.h"
void minimize(MetricsData data)
{
    // nlopt_opt opt = nlopt_create(NLOPT_LD_SLSQP, data.n); // создаем объект оптимайзера
    // double lower_bounds[data.n];                          // нижние границы для переменных - ноль для изменения по времени и минус бесконечность для ускорения
    // for (int i = 0; i < data.n; i++)
    // {
    //     if (i % 2)
    //         lower_bounds[i] = 0;
    //     else
    //         lower_bounds[i] = -1e20;
    // }
    // void *data_ptr = static_cast<void *>(&data);                 // переводим дату в войд для передачи в оптимайзер
    // nlopt_set_lower_bounds(opt, lower_bounds);                   // собственно задаем нижние границы
    // nlopt_set_min_objective(opt, metrics::minimizing, data_ptr); // задаем функцию для минимизации
    // double tolerances[3] = {1e-8, 1e-8, 1e-8};
    // nlopt_add_equality_mconstraint(opt, 3, metrics::constraints, data_ptr, tolerances); // задаем функцию ограничений
    // nlopt_set_ftol_rel(opt, 1e-4);                                                      // abs??  x??          - задаем критерий остановки - относительное изменение метрики меньше 1е-4 (мб сделать для иксов? или абсолютное?)

    double resultCon[3], gradientCon[3 * data.n], x[data.n], gradientMin[data.n], resultMin;
    Point Vm = bangbang(data.vel, data.endVel, data.endPos - data.pos, MAX_ACC, MAX_VEL);
    // std::cout<<Vm.x<<" "<<Vm.y<<"\n";
    // double acc_ang = (Vm - data.vel).arg();
    // double dec_ang = (data.endVel - Vm).arg();
    double acc_time = (data.vel - Vm).mag() / MAX_ACC;
    double dec_time = (data.endVel - Vm).mag() / MAX_ACC;
    double const_time = ((data.endPos - data.pos) - (data.vel + Vm) / 2 * acc_time - (data.endVel + Vm) / 2 * dec_time).mag() / Vm.mag();
    if (const_time < EPSILON)
        const_time = 0;
    // double T = acc_time + dec_time + const_time;

    for (int i = 0; i < data.n / 2; i++)
    {
        if (const_time > 0)
            x[i * 2] = Vm.arg();
        else
        {
            double angle = vecAux::getAngleBetweenPoints(data.vel, Point(0, 0), (Vm - data.vel));
            double cA = cos(angle);
            double l = cA * data.vel.mag() + sqrt(cA * cA * data.vel.mag2() - (data.vel.mag2() - MAX_VEL * MAX_VEL));
            double beta = asin(l / MAX_VEL * sin(angle));
            x[i * 2] = data.vel.arg() - beta;
        }
        x[i * 2 + 1] = (acc_time + const_time) / data.n * 2;
    }
    void *voidData = static_cast<void *>(&data);

    metrics::constraints(3, resultCon, data.n, x, gradientCon, voidData);

    // double minf;
    // nlopt_optimize(opt, data.x, &minf);
    // std::cout << minf << " " << data.resultCon[0] << " " << data.resultCon[1] << " " << data.resultCon[2] << "\n";
    // nlopt_destroy(opt);
}