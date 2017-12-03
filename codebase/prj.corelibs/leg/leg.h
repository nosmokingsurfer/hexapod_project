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


using namespace std;
using namespace Eigen;

class Leg{
public: 
  Leg();
  ~Leg();

 bool init(const vector<double>& segments, const Vector4d& mountingPoint, const Vector3d& mountingAngles);
 bool init(const Eigen::Vector3d& segments, const Vector4d& mountingPoint, const Vector3d& mountingAngles);
  
public:
  vector<double> segments;
  void inverseKinematics(Vector3d& targetPoint, Vector3d& solution);
  void forwardKinematics(Vector3d& targetAngles, Vector3d& solution);
  bool checkReachability(Vector3d& targetPoint);
  Eigen::Vector3d trajectoryGenerator(double time);
private:
  Matrix4d mounting; //leg position and orientation in parent reference frame
  vector<double> state; //leg state - array of joint angles
};

#endif