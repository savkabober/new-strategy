/*
великие алгоритмы минимизации
*/

#pragma once
#include <nlopt.hpp>
#include "metrics/metrics.h"
MetricsData mimimize(MetricsData data)
{
    nlopt::opt opt(nlopt::LD_SLSQP, data.n);//создаем объект оптимайзера
    double lower_bounds[data.n];//нижние границы для переменных - ноль для изменения по времени и минус бесконечность для ускорения
    for(int i = 0;i<data.n;i++)
    {
        if(i%2)
            lower_bounds[i] = 0;
        else
            lower_bounds[i] = -HUGE_VAL;
    }
    void *data_ptr = static_cast<void *>(&data);// переводим дату в войд для передачи в оптимайзер
    nlopt_set_lower_bounds(opt,lower_bounds);//собственно задаем нижние границы
    nlopt_set_min_objective(opt,metrics::minimizing,data_ptr);//задаем функцию для минимизации
    nlopt_add_equality_constraint(opt,metrics::constraints,data_ptr,1e-8);//задаем функцию ограничений
    nlopt_set_ftol_rel(opt,1e-4);//abs??  x??          - задаем критерий остановки - относительное изменение метрики меньше 1е-4 (мб сделать для иксов? или абсолютное?)
    

    return 0;
}