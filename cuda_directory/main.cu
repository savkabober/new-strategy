#include "../include/const.h"
#include "stdio.h"
#include "cuda_include/cuda_polynome.cuh"
#include <cstdlib>
#include <ctime>

int main(void)
{
    double roots[4];
    // int n = solve_square(1,-3,2,roots);
    // int n = solve_qubic(1,2,3,9,roots);
    float A,B,C,D,E,x;
    A = 1;
    B = 2;
    C = -6;
    D = -5;
    E = 2;
    int n = solve_quadric(A,B,C,D,E,roots);
    printf("n = %i\n",n);
    // for(int i = 0;i<n;i++)
    // {
    //     x = roots[i];
    //     printf("%f \n",roots[i]);
    //     // printf("%f\n",A*x*x*x*x+B*x*x*x+C*x*x+D*x+E);
    // }

    // int n,i;
    // float A,B,C,D,E,x;
    // for(int j = 0;j<100;j++)
    // {
    //     A = 10*(float)rand() / RAND_MAX;
    //     B = 10*(float)rand() / RAND_MAX;
    //     C = 10*(float)rand() / RAND_MAX;
    //     D = 10*(float)rand() / RAND_MAX;
    //     E = 10*(float)rand() / RAND_MAX;
    //     n = solve_quadric(A,B,C,D,E,roots);
    //     for(i = 0;i<n;i++)
    //     {
    //         x = roots[i];
    //         printf("%f\n",A*x*x*x*x+B*x*x*x+C*x*x+D*x+E);
    //     }
    //     printf("\n");
    // }
    return 0;
}