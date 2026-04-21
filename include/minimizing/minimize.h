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
    //         lower_bounds[i] = -HUGE_VAL;
    // }
    // void *data_ptr = static_cast<void *>(&data);                 // переводим дату в войд для передачи в оптимайзер
    // nlopt_set_lower_bounds(opt, lower_bounds);                   // собственно задаем нижние границы
    // nlopt_set_min_objective(opt, metrics::minimizing, data_ptr); // задаем функцию для минимизации
    // double tolerances[3] = {1e-8, 1e-8, 1e-8};
    // nlopt_add_equality_mconstraint(opt, 3, metrics::constraints, data_ptr, tolerances); // задаем функцию ограничений
    // nlopt_set_ftol_rel(opt, 1e-4);                                                      // abs??  x??          - задаем критерий остановки - относительное изменение метрики меньше 1е-4 (мб сделать для иксов? или абсолютное?)

    Point Vm = bangbang(data.vel, data.endVel, data.endPos - data.pos, MAX_ACC, MAX_VEL);
    double acc_ang = (Vm - data.vel).arg();
    double dec_ang = (data.endVel - Vm).arg();
    double acc_time = (data.vel - Vm).mag() / MAX_ACC;
    double dec_time = (data.endVel - Vm).mag() / MAX_ACC;
    double const_time = ((data.endPos - data.pos) - (data.vel + Vm) / 2 * acc_time - (data.endVel + Vm) / 2 * dec_time).mag() / Vm.mag();
    if (const_time < EPSILON)
        const_time = 0;
    double T = acc_time + dec_time + const_time;

    for (int i = 0; i < data.n / 2; i++)
    {
        if(const_time>0)
            data.x[i*2] = Vm.arg();
        else
        {
            double angle = vecAux::getAngleBetweenPoints(data.vel,Point(0,0),(Vm-data.vel));
            double l = cos(angle)*data.vel.mag()+sqrtf(cos(angle)*cos(angle)*data.vel.mag2()-(data.vel.mag2()-MAX_VEL*MAX_VEL));
            double beta = asin(l/MAX_VEL*sin(angle));
            data.x[i*2] = data.vel.arg()-beta;
        }
        data.x[i*2+1] = (acc_time+dec_time)/data.n*2;
    }

    // int swithcId = std::trunc((acc_time + const_time) / (T / data.n * 2));
    // data.x[swithcId * 2] = dec_ang;
    // data.x[swithcId * 2 + 1] = T / data.n * 2 - fmod((acc_time + const_time), (T / data.n * 2));
    // data.x[swithcId * 2 - 1] = T / data.n * 2 + fmod((acc_time + const_time), (T / data.n * 2));

    // double minf;
    // if (nlopt_optimize(opt, data.x, &minf) < 0)
    // {
    //     std::cerr << "Ошибка NLopt!" << std::endl;
    // }
    // else
    // {
    //     std::cout << "Найдено решение:\n";
    //     std::cout << "f = " << minf << std::endl;
    // }

    // nlopt_destroy(opt);
}