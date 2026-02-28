/*
Класс таймера (начало времени - создание объекта)
*/
#pragma once

#include <chrono>

using namespace std;

class Timer {
    private:
    chrono::time_point<chrono::high_resolution_clock> start_clock_time;
    public:
    Timer() : start_clock_time(chrono::high_resolution_clock::now()) {};
    long double time() {
        chrono::time_point<chrono::high_resolution_clock> now_time;
        now_time = chrono::high_resolution_clock::now();
        chrono::duration<long double> delta = now_time - start_clock_time;
        return delta.count();
    }
    void reset() {
        start_clock_time = chrono::high_resolution_clock::now();
    }
};