///
///@file hexapodclasses/leg.h
///@authors Panchenko A.V.
///@brief Leg class for inverse kinematics and etc
///


#pragma once
#ifndef LEG_H
#define LEG_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>

#include <pose/pose.h>
#include <pid/pid.h>

using namespace std;
using namespace Eigen;

class Leg{
public: 
  Leg();
  ~Leg();

 bool init(const std::string name, const Eigen::Vector3d& segments, const Vector3d& mountingPoint, const Vector3d& mountingAngles);
 bool init(const std::string name, const vector<double>& segments, const Eigen::Vector3d& mountingPoint, const Eigen::Vector3d& mountingAngles);
public:
  vector<double> segments;
  Eigen::Vector3d inverseKinematics(const Vector3d& targetPoint);
  Eigen::Vector3d forwardKinematics(const Vector3d& targetAngles);
  bool checkReachability(const Vector3d& targetPoint);
  bool recieveFB(const VectorXd& feedback, int index);
  bool setCoeffs(const PID::PIDcoeffs &coeffs);


  Eigen::Vector3d trajectoryGenerator(double time);
  Eigen::VectorXd getTorques(const VectorXd& targetAngles);

  std::vector<PID> pids; // all PID controllers of the leg
  
  Pose pose;//leg mounting pose

private:
  std::string legName; //leg name
  Vector3d FBcoords; //leg state - array of joint angles
  Vector3d FBvelocities; //leg coordinate velocities
  Vector3d targetState; // current leg target position
};

#endif