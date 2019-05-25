#ifndef FRENET_PLANNER_HPP
#define FRENET_PLANNER_HPP

#include <algorithm>
#include <cfloat>
#include "../include/polynomials.hpp"
#include "../include/cubic_spline_planner.hpp"
#include <tf/transform_listener.h>		
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>			

// Parameter
double MAX_SPEED; // maximum speed [m/s]
double MAX_ACCEL;  // maximum acceleration [m/ss]
double MAX_CURVATURE;  // maximum curvature [1/m]
double MAX_ROAD_WIDTH;  // maximum road width [m]
double D_ROAD_W;  // road width sampling length [m]
double DT;  // time tick [s]
double MAXT;  // max prediction time [m]
double MINT;  // min prediction time [m]
double TARGET_SPEED;  // target speed [m/s]
double D_T_S;   // target speed sampling length [m/s]
double N_S_SAMPLE;// sampling number of target speed
double ROBOT_RADIUS;  // robot radius [m]

// cost weights
double KJ;
double KT;
double KD;
double KLAT;
double KLON;

nav_msgs::Odometry odom;
nav_msgs::OccupancyGrid cmap;
geometry_msgs::PolygonStamped footprint;
vector<double> ob_x;
vector<double> ob_y;


class FrenetPath{
	public :

		vecD t, d, d_d, d_dd, d_ddd, s, s_d, s_dd, s_ddd, x, y, yaw, ds, c;
		double cd, cv, cf;


};

vector<FrenetPath> calc_frenet_paths(double, double, double, double, double);
vector<FrenetPath> calc_global_paths(vector<FrenetPath> &, double);
vector<FrenetPath> check_path(vector<FrenetPath>	);
FrenetPath frenet_optimal_planning(Spline2D, double, double, double, double, double);

bool check_collision(FrenetPath);

#endif
