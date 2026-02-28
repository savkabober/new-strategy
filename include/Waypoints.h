#include <baseInclude.h>

using namespace std;

struct Waypoint
{
    double acc_angle, time;
    Waypoint(double acc_angle_, double time_) : acc_angle(acc_angle_), time(time_) {}
};

using Waypoints = vector<Waypoint>;