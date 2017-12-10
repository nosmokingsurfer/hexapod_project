///
///@file body/body.h
///@authors Panchenko A.V.
///@brief Body class for robot body
///


#pragma once
#ifndef BODY_H
#define BODY_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <leg/leg.h>
#include <pose/pose.h>


#include <vector>
#include <iostream>


using namespace std;
using namespace Eigen;

class Body{
public: 
  Body();
  ~Body();
public:
  std::vector<Leg> legs; // legs connected to body
  //Pose fbPose; // coordinates of boy
  VectorXd FBcoords;
  VectorXd FBvelocities;
  int fbIndex;//index in FB array
  
  bool getFB(VectorXd& feedback);// get feedback from UM
  VectorXd getTargetPosition(double time); // get target position in global RF

};

#endif