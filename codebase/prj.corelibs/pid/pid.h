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
  struct PIDcoeffs{
    double kP;
    double kI;
    double kD;
  };
public: 
   PID();
   PID(const double kP, const double kI, const double kD);
   PID(const Vector3d coeffs);
  ~PID();
private:
  PIDcoeffs coeffs;

  double P;
  double I;
  double D;

public:
  bool setCoeffs(const PIDcoeffs& coeffs);
  bool setCoeffs(const Vector3d& coeffs);
  bool setCoeffs(const double kP, const double kI, const double kD);
  bool setState(const Vector3d& state);

  bool updateState(double x, double dot_x);

  double getValue(double x_0, double x, double dot_x);

  bool reset();
};

#endif