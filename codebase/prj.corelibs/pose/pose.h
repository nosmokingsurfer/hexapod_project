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
  Pose(Vector3d t);
  Pose(Vector3d t, MatrixXd R);
  ~Pose();

private:
  MatrixXd R;
  Vector3d t;

public:
  MatrixXd getOrientationMatrix();
  Vector3d getPosition();

  MatrixXd getCoordsInPoseReferenceFrame(MatrixXd externalCoords);
  MatrixXd getCoordsOutsidePoseReferenceFrame(MatrixXd internalCoords);

  Pose inverse();

};

#endif