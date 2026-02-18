#include "../../include/const.h"
typedef unsigned char uint8_t;
__host__ __device__ int solve_square(double a, double b, double c, double roots[2])
{
    double D = b*b - 4*a*c;
    if(D>EPSILON)
    {
        roots[0] = (-b+sqrt(D))/(2*a);
        roots[1] = (-b-sqrt(D))/(2*a);
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
__host__ __device__ int solve_qubic(double a, double b, double c, double d, double roots[3])
{
    double B = b/a;
    double C = c/a;
    double D = d/a;
    
    double p = C-B*B/3;
    double q = D-B*C/3+2*B*B*B/27;
    double Discr = q*q/4 + p*p*p/27;
    double y1;
    if(Discr>EPSILON)
    {
        y1 = cbrt(-q/2+sqrt(Discr))+cbrt(-q/2-sqrt(Discr));
    }
    if(abs(Discr)<EPSILON)
    {
        y1 = 2*cbrt(-q/2);
    }
    if(Discr<=EPSILON)
    {
        double O = acos((-q/2)/sqrt(-p*p*p/27));
        y1 = 2*sqrt(-p/3)*cos(O/3);
    }
    double x = y1-B/3;
    roots[0] = x;
    double B1 = B+x;
    double C1 = C+B*x+x*x;
    double rootsx[2];
    int n = solve_square(1,B1,C1,rootsx);
    for(int i = 0;i<n;i++)
    {
        roots[i+1] = rootsx[i];
    }
    // for(int i = 0;i<n+1;i++)
    //     printf("%f\n",roots[i]*roots[i]*roots[i]+B*roots[i]*roots[i]+C*roots[i]+D);
    return 1+n;
}
__host__ __device__ int solve_quadric(double a, double b, double c, double d, double e, double roots[4])
{
    if(fabs(a) <EPSILON)
        return solve_qubic(b,c,d,e,roots);
    double A = b/a;
    double B = c/a;
    double C = d/a;
    double D = e/a;

    double p = B-3*A*A/8; 
    double q = A*A*A/8 - A*B/2 + C;
    double r = -3*A*A*A*A/256 + A*A*B/16 - C*A/4 + D;
    // printf("pqr: %f, %f, %f\n",p,q,r);
    double rootss[3];
    solve_qubic(2,-p,-2*r,r*p-q*q/4,rootss);
    double s = rootss[0];
    // printf("s: %f",s);
    // printf(" %f",2*s*s*s -p*s*s -2*r*s +r*p-q*q/4);

    double rootsy1[2];
    double rootsy2[2];
    uint8_t n1 = solve_square(1,-sqrt(2*s-p),q/2/sqrt(2*s-p)+s,rootsy1);
    uint8_t n2 = solve_square(1,sqrt(2*s-p),-q/2/sqrt(2*s-p)+s,rootsy2);
    for(uint8_t i = 0;i<n1;i++)
        roots[i] = rootsy1[i]-A/4;
    for(uint8_t i = 0;i<n2;i++)
        roots[i+n1] = rootsy2[i]-A/4;
    return n1+n2;    
}