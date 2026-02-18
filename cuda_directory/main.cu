#include "../include/const.h"
#include "stdio.h"
#include "cuda_include/cuda_polynome.cuh"
#include <cstdlib>
#include <chrono>
#include <stdio.h>
long long get_time_ms() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    return millis;
}
int main(void)
{
    double roots[4];
    double A = 1,B = 4,C = -4,D = -20,E = -5;
    int n;
    double cnt,x,res;
    // n = solve_quadric(A,B,C,D,E,roots);
    // printf("%i\n",n);
    // for(int i = 0;i<n;i++)
    //     {
    //         x = roots[i];
    //         printf("%f  ",x);
    //         res = A*x*x*x*x+B*x*x*x+C*x*x+D*x+E;
    //         printf(" %f\n",res);
    //         if(res>cnt)
    //             cnt = res;
    //     }

    srand(static_cast<unsigned int>(time(NULL)));
    long long timer = get_time_ms();
    for(int z = 0;z<100000;z++)
    {
        A =(double)rand()/RAND_MAX*100-50;
        B =(double)rand()/RAND_MAX*100-50;
        C =(double)rand()/RAND_MAX*100-50;
        D =(double)rand()/RAND_MAX*100-50;
        E =(double)rand()/RAND_MAX*100-50;
        n = solve_quadric(A,B,C,D,E,roots);
        // printf("%f, %f, %f, %f,n =  %i roots: ",A,B,C,D,n);
        for(int i = 0;i<n;i++)
        {
            x = roots[i];
            // printf("%f ",x);
            res = A*x*x*x*x+B*x*x*x+C*x*x+D*x+E;
            // printf("%f\n",res);
            if(res>cnt)
                cnt = res;
        }
        // printf("\n");
    }
    printf("\n%lli\n",get_time_ms()-timer);
}