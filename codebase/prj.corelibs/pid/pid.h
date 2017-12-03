///
///@file pid/pid.h
///@authors Panchenko A.V.
///@brief PID controller class
///


#pragma once
#ifndef PID_H
#define PID_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>


using namespace std;
using namespace Eigen;

class PID{
public: 
   PID();
   PID(double kP, double kI, double kD);
   PID(Vector3d coeffs);
  ~PID();
private:
  double kP;
  double kI;
  double kD;

  double P;
  double I;
  double D;

public:
  bool setCoeefs(Vector3d& coeffs);
  bool setCoeefs(double kP, double kI, double kD);
  bool setState(Vector3d& state);

  bool updateState(double x, double dot_x);

  double getValue(double x_0, double x, double dot_x);

  bool reset();
};

#endif