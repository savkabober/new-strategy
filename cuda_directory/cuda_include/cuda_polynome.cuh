#include "../../include/const.h"
__host__ __device__ int solve_square(float a, float b, float c, float roots[2])
{
    float D = b*b - 4*a*c;
    if(D>EPSILON)
    {
        roots[0] = (-b+sqrtf(D))/(2*a);
        roots[0] = (-b-sqrtf(D))/(2*a);
        return 2;
    }
    else if(abs(D)<=EPSILON)
    {
        roots[0] = (-b)/(2*a);
        return 1;
    }
    else
        return 0;
}
