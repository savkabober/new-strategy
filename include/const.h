#pragma once

//robot movement
#define MAX_SPEED 1500
#define MAX_ACC 1500
//idk
#define MAX_ROBOT_COUNT 16
#define MAX_SOLVER_DEGREE 10
#define TIME_TO_BORN 0.0
#define TIME_TO_DIE 1.5
//geometry
#define ROBOT_R 100.0
#define BALL_R 40.0
#define GRAVEYARD_POS_X 10000
#define FIELD_DX 9000
#define FIELD_DY 6000
#define ZONE_DX 1000
#define ZONE_DY 2000
#define GOAL_DY 1000
#define POLARITY 1
#define FIELD_COLOR Color(52, 92, 10)
#define FIELD_MARGIN 200
//field info
#define ALL_TEAMS 0
#define BLUE 1
#define YELLOW 2
#define COLOR 1
#define POLARITY 1 //sign of x coordinate of our goal
//metrics
#define DIST_K 1e6
#define VEL_K 1e4
#define OBSTACLE_K 1e2
#define POW_BASE 1.003
//some math constants
#define EPSILON 1e-10
#define INF 1e30
//physycs
// #define DRIBBLER_R 0.006
// #define BALL_R 0.02133
// #define DRIBBLER_MU 0.5
// #define FLOOR_MU 0.25
// #define ALPHA 47.87
// #define BALL_L 0.111
// #define g Point3D(0,0,-9.81)
// Point3D R_bd(0,-sin(ALPHA)*BALL_R,cos(ALPHA)*BALL_R);
// Point3D R_db = Point3D()-R_bd.unity()*DRIBBLER_R;
// Point3D R_bf = Point3D(0,0,-BALL_R);
