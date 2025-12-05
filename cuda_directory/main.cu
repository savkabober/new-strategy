#include "../include/const.h"
#include "stdio.h"
#include "cuda_include/cuda_polynome.cuh"
int main(void)
{
    float roots[2];
    int n = solve_square(1,2,1,roots);
    for(int i = 0;i<n;i++)
    {
        printf("%f \n",roots[i]);
    }
    return 0;
}