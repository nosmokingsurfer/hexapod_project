///
///@file trajectory/trajectory.h
///@authors Panchenko A.V.
///@brief Trajectory class to work with trajectories
///


#pragma once
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>

#include <pose/pose.h>


using namespace std;
using namespace Eigen;

class Trajectory{
public: 
  Trajectory();
  ~Trajectory();

private:
  vector<Pose> controlPoints;
public:
  bool init(const vector<Pose> &controlPoints);
  double getCTE(const Pose& curPose);
  double deltaPose(const Pose& curPose);
  Pose interpolate(const Pose& p1, const Pose& p2, const double lambda);
  
};

#endif