///
///@file robot/robot.h
///@authors Panchenko A.V.
///@brief Robot class for high level control
///

#include <leg/leg.h>

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
public: 
  Eigen::VectorXd getControls();
  Eigen::VectorXd getControls(double time);

};

#endif