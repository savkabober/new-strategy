#include <random>
#include <chrono>
#include <thread>

using namespace std;

namespace quick_random {
    double random_double(double min, double max) {
        mt19937 double_seed(chrono::high_resolution_clock::now().time_since_epoch().count());
        return uniform_real_distribution<double>(min, max)(double_seed);
    }

    int random_int(int min, int max) {
        mt19937 int_seed(chrono::high_resolution_clock::now().time_since_epoch().count());
        return uniform_int_distribution<int>(min, max)(int_seed);
    }
}