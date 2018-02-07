///
///@file pose/pose.h
///@authors Panchenko A.V.
///@brief Pose class - for working with coordinate transformations
///


#pragma once
#ifndef POSE_H
#define POSE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>



using namespace Eigen;

class Pose{
public: 
  Pose();
  Pose(Vector3d angles);
  Pose(Vector3d angles, Vector3d t);
  Pose(const Transform<double, 3, Affine>& T);
  ~Pose();  

public:
  Transform<double, 3, Affine> T;

public:
  MatrixXd getRotation();
  Vector3d getTranslation();

  bool setPosition(Eigen::Vector3d position);
  void getAngles();

  static Pose getRotationAroundAxis(Vector3d axis, double angle);
  Pose operator*(Pose);
  Pose inverse();

};

#endif