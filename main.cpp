#include "include/numAux.h"
#include "include/Timer.h"

int main()
{
    double prod[4], time;
    Timer timer;
    timer.reset();
    for (long i = 0; i < 1e6; i++) {
        numAux::solveEq(prod, 1, -21, 148, -384, 256);
    }
    time = timer.time();
    cout << time << endl;
    return 0;
}
