#include "../../include/const.h"
#include "cuda_polynome.cuh"
struct  Point
{
    double x,y;
    __host__ __device__ Point(double x_ = 0,double y_ = 0):x(x_),y(y_){}
    __host__ __device__ Point operator+(const Point& other)
    {
        return Point(x+other.x,y+other.y);
    }
    __host__ __device__ Point operator-(const Point& other)
    {
        return Point(x-other.x,y-other.y);
    }
    __host__ __device__ Point operator*(const double& other)
    {
        return Point(x*other,y*other);
    }
    __host__ __device__ Point operator/(const double& other)
    {
        return Point(x/other,y/other);
    }
    __host__ __device__ float mag()
    {
        return sqrtf(x*x+y*y);
    }
    __host__ __device__ Point unity()
    {
        return Point(x,y)/this->mag();
    }
    __host__ __device__ Point rotate(double angle)
    {
        return Point(x * cos(angle) - y * sin(angle),y * cos(angle) + x * sin(angle));
    }
};
