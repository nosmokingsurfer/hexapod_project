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
#include <pid/pid.h>
#include <pose/pose.h>
#include <segment/segment.h>


#include <vector>
#include <iostream>


using namespace std;
using namespace Eigen;

class Body{
public: 
  Body();
  ~Body();
public:
  vector<Segment> segments;
  Pose fbPose; //!< feedback pose of the robot's body
  Pose tarPose; //!< target pose of the robot's body

  VectorXd FBcoords;
  VectorXd FBvelocities;
  int fbIndex; //index in FB array
  
  bool recieveFB(VectorXd& feedback);// get feedback from UM
  Pose getTargetPose(double time); // get target position of body in global RF

  void printOut();//!< print all the element of the mozaik body into the console
};


#endif