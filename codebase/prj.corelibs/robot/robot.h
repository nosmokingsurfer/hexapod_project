///
///@file robot/robot.h
///@authors Panchenko A.V.
///@brief Robot class for high level control
///

#include <leg/leg.h>
#include <pid/pid.h>

#pragma once
#ifndef ROBOT_H
#define ROBOT_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>


using namespace std;
using namespace Eigen;

class Robot{
public: 
  Robot();
  ~Robot();
  private:
    std::vector<Leg> robotLegs;
    VectorXd feedBack; // the whole feedback array from UM
    VectorXd FBcoords; // coords of the robot body
    VectorXd FBvelocities; // velocities of the robot body
    VectorXd controlTorques; // the whole output array from control system
    VectorXd calculatedjoints; // calculated join angles
    VectorXd parameters; // the whole list of parameters coming from UM

public: 
  Eigen::VectorXd getControls();
  Eigen::VectorXd getControls(double time);
  Eigen::VectorXd getCalculatedJoints();

  bool recieveFeedBack(double* inputs, int numberOfInputs);
  bool recieveParameters(double* params, int numberOfParams);

  bool getFB(int index);

  //bool writeSelf();

};

#endif