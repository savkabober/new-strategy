/*
Удобное получения псевдослучайных чисел
*/
#pragma once

#include <random>
#include <chrono>
#include <thread>

using namespace std;

namespace quickrandom
{
    double randomDouble(double min, double max)
    {
        mt19937 double_seed(chrono::high_resolution_clock::now().time_since_epoch().count());
        return uniform_real_distribution<double>(min, max)(double_seed);
    }

    double random()
    {
        return randomDouble(0, 1);
    }

    int randomInt(int min, int max)
    {
        mt19937 int_seed(chrono::high_resolution_clock::now().time_since_epoch().count());
        return uniform_int_distribution<int>(min, max)(int_seed);
    }
}