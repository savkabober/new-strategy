#pragma once
#include "../../include/const.h"
#include "cuda_polynome.cuh"
#include "cuda_auxiliary.cuh"
struct Waypoint
{
    double acc_angle;
    double time;
    Waypoint(double _acc = 0, double _time = 0):acc_angle(_acc), time(_time) {}
    Point get_acc()
    {
        return Point(MAX_ACC,0).rotate(acc_angle);
    }
};
struct est_params
{
    Point start_pos, start_vel, end_pos, end_vel;
    int obstacle_count;
    Point* obstacles;
    est_params(Point _start_pos = Point(0,0),Point _start_vel = Point(0,0),Point _end_pos = Point(0,0),Point _end_vel = Point(0,0), int _obstacle_count, Point* _obstacles):start_pos(_start_pos),start_vel(_start_vel),end_pos(_end_pos),end_vel(_end_vel), obstacle_count(_obstacle_count),obstacles(_obstacles){}

};
double est_pos(int traj_len, Waypoint* Trajectory, est_params pars)
{
    Point cur_pos = pars.start_pos;
    Point cur_vel = pars.start_vel;
    for(int i = 0;i<traj_len;i++)
    {
        Waypoint cur_wp = Trajectory[i];
        Point end_vel;
        if(cur_vel.mag()<MAX_SPEED)
        {
            if((cur_vel+cur_wp.get_acc()).mag()>MAX_SPEED)
            {
                double rootst[2];
                int n = solve_square(cur_wp.get_acc().mag()*cur_wp.get_acc().mag(),2*(cur_vel.x*cur_wp.get_acc().x+cur_vel.y*cur_wp.get_acc().y),cur_vel*cur_vel-MAX_SPEED*MAX_SPEED,rootst);
                double t = cur_wp.time;
                for(int i = 0;i<n;i++)
                    if(rootst[i] > 0 && rootst[i]<cur_wp.time)
                        t = rootst[i];
                cur_pos = cur_pos+cur_vel*t+cur_wp.get_acc()*t*t;
                cur_vel = cur_vel+cur_wp.get_acc()*t;
            }
            else
            {
                cur_pos = cur_pos+cur_vel*cur_wp.time+cur_wp.get_acc()*cur_wp.time*cur_wp.time;
                cur_vel = cur_vel+cur_wp.get_acc()*cur_wp.time;
            }
        }
    }
    return (pars.end_pos-cur_pos).mag()+(pars.end_vel-cur_vel).mag();
}
double est_time(int traj_len, Waypoint* Trajectory, est_params pars)
{
    double T;
    for(int i = 0;i<traj_len; i++)
        T+= Trajectory[i].time;
    return T;
}
double est_obstacles(int traj_len, Waypoint* Trajectory, est_params pars)
{
    double est;
    for(int i = 0;i<traj_len;i++)
    {
        Point S;
        Point V;
        Point A;
        Waypoint cur_wp = Trajectory[i];
        for(int j = 0;i<pars.obstacle_count;j++)
        {
            Point E = pars.obstacles[j];
            float R = ROBOT_R;
            double rootst[4];
            int n = solve_quadric(A.mag()*A.mag(),2*(V.x*A.x+V.y*A.y),V.mag()*V.mag()+2*(S.x*A.x+S.y*A.y+E.x*A.x+E.y*A.y),2*(S.x*V.x+S.y*V.y+V.x*E.x+V.y*E.y),);
        }
    }

    return est;
}