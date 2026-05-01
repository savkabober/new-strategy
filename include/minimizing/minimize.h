/*
великие алгоритмы минимизации
*/

#pragma once
#include <nlopt.h>
#include "../metrics/metrics.h"
#include "initialApprox.h"

void minimize(MetricsData data)
{
    nlopt_opt opt = nlopt_create(NLOPT_LD_SLSQP, data.n); // создаем объект оптимайзера
    double lower_bounds[data.n];                          // нижние границы для переменных - ноль для изменения по времени и минус бесконечность для ускорения
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
    // nlopt_set_check_gradients(opt, 1);          // 1 = включить
    // nlopt_set_check_gradient_step(opt, 1e-5);   // шаг для численного сравнения

    double resultCon[3], gradientCon[3 * data.n], x[data.n], gradientMin[data.n], resultMin;
    Point Vm = bangBang::bangBang(data.vel, data.endVel, data.endPos - data.pos, MAX_ACC, MAX_VEL);
    // std::cout<<Vm.x<<" "<<Vm.y<<"\n";
    // double acc_ang = (Vm - data.vel).arg();
    // double dec_ang = (data.endVel - Vm).arg();
    double acc_time = (data.vel - Vm).mag() / MAX_ACC;
    double dec_time = (data.endVel - Vm).mag() / MAX_ACC;
    double const_time = ((data.endPos - data.pos) - (data.vel + Vm) / 2 * acc_time - (data.endVel + Vm) / 2 * dec_time).mag() / Vm.mag();
    if (const_time < EPSILON)
        const_time = 0;

    for (int i = 0; i < data.n / 4; i++)
    {
        if (const_time > 0)
            x[i * 2] = Vm.arg();
        else
        {
            double angle = vecAux::getAngleBetweenPoints(data.vel, Point(0, 0), (Vm - data.vel));
            double cA = cos(angle);
            double l = -cA * data.vel.mag() + sqrt(cA * cA * data.vel.mag2() - (data.vel.mag2() - MAX_VEL * MAX_VEL));
            x[i * 2] = (data.vel + (Vm - data.vel).unity() * l).arg();
        }
        x[i * 2 + 1] = (acc_time + const_time) / (data.n / 4);
        // cout << x[2 * i] << " " << x[2 * i + 1] << endl;
    }
    for (int i = data.n / 4; i < data.n / 2 - 1; i++)
    {
        double angle = vecAux::getAngleBetweenPoints(Vm, Point(0, 0), (data.endVel - Vm));
        double cA = cos(angle);
        double l = -cA * Vm.mag() + sqrt(cA * cA * Vm.mag2() - (Vm.mag2() - MAX_VEL * MAX_VEL));
        x[i * 2] = (Vm + (data.endVel - Vm).unity() * l).arg();
        x[i * 2 + 1] = dec_time / (data.n / 2 - data.n / 4);
        // cout << Vm + (data.endVel - Vm).unity() * l << " " << x[2 * i + 1] << endl;
    }
    // x[data.n - 2] = 0;
    // x[data.n - 1] = 0.5;
    // cout << data.vel << " " << Vm << " " << data.endVel << endl;

    // void *voidData = static_cast<void *>(&data);
    // metrics::constraints(3, resultCon, data.n, x, gradientCon, voidData);

    double minf;
    // cout << "lol" << endl;
    int result = nlopt_optimize(opt, x, &minf);
    std::cout << result << " " << minf << " " << data.resultCon[0] << " " << data.resultCon[1] << " " << data.resultCon[2] << "\n";
    nlopt_destroy(opt);
}