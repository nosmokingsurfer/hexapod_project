///
///@file robot/robot.h
///@authors Panchenko A.V.
///@brief Robot class for high level control
///



#pragma once
#ifndef ROBOT_H
#define ROBOT_H

#include <leg/leg.h>
#include <pid/pid.h>
#include <body/body.h>


#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>


using namespace std;
using namespace Eigen;

class Robot{
public: 
  Robot();
  Robot(Body::BODY_TYPE bt);
  ~Robot();
  private:
    Body robotBody; // robot body.

    VectorXd feedBack; // the whole feedback array from UM

    VectorXd controlTorques; // the whole output array from control system

    VectorXd calculatedjoints; // calculated join angles

    VectorXd parameters; // the whole list of parameters coming from UM

public: 
  Eigen::VectorXd getControls(double time);
  Eigen::VectorXd getCalculatedJoints();

  bool recieveFeedBack(double* inputs, int numberOfInputs);
  bool recieveParameters(double* params, int numberOfParams);
};

#endif