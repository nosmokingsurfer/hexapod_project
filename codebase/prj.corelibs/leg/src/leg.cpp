//#include "stdafx.h"
#include <leg/leg.h>


Leg::Leg()
{
  this->segments.clear();
}


Leg::~Leg()
{

}

bool Leg::init(const std::string name, const vector<double>& segments, const Eigen::Vector3d& mountingPoint, const Eigen::Vector3d& mountingAngles)
{
  for (size_t i = 0; i < segments.size(); i++)
  {
    this->segments.push_back(segments[i]);
  }

  pose = Pose(mountingAngles, mountingPoint);

  //PID controller for all joints
  pids.push_back(PID(Vector3d(0,0,0)));
  pids.push_back(PID(Vector3d(0,0,0)));
  pids.push_back(PID(Vector3d(0,0,0)));


  return true;
}

bool Leg::init(const std::string name, const Eigen::Vector3d& segments, const Vector3d& mountingPoint, const Vector3d& mountingAngles)
{
  std::vector<double> segms;
  segms.push_back(segments[0]);
  segms.push_back(segments[1]);
  segms.push_back(segments[2]);

  return this->init(name, segms, mountingPoint ,mountingAngles);
}

Eigen::Vector3d Leg::inverseKinematics(const Eigen::Vector3d& targetPoint)
{
  double p1 = this->segments[0];
  double p2 = this->segments[1];
  double p3 = this->segments[2];

  double x = targetPoint[0];
  double y = targetPoint[1];
  double z = targetPoint[2];
  
  
  double alpha = 0;
  double beta = 0;
  double gamma = 0;

  if (!checkReachability(targetPoint))
    return Eigen::Vector3d(0,0,0);

  alpha = atan2(y, x);
  double x_cap = sqrt(x*x + y*y);
  double l = sqrt(z*z + (x_cap - p1)*(x_cap - p1));

  double psi1 = atan2(z, x_cap - p1);

  double tempArg = (p2*p2 + l*l - p3*p3)/(2*p2*l);
  if (tempArg < -1.0) tempArg = -1.0;
  else if(tempArg > 1.0) tempArg = 1.0;

  double psi2 = acos(tempArg);  

  beta = psi1 + psi2;

  tempArg = (l*l - p2*p2 - p3*p3)/(-2*p2*p3);
  if (tempArg < -1.0) tempArg = -1.0;
  else if(tempArg > 1.0) tempArg = 1.0;
  
  gamma = acos(tempArg) - EIGEN_PI;

  return Eigen::Vector3d(alpha, beta, gamma);
  
}


Eigen::Vector3d Leg::forwardKinematics(const Eigen::Vector3d& targetAngles)
{
  Eigen::Vector3d solution;

  double p1 = this->segments[0];
  double p2 = this->segments[1];
  double p3 = this->segments[2];

  double alpha = targetAngles[0];
  double beta  = targetAngles[1];
  double gamma = targetAngles[2];

  double x = cos(alpha)*(p1 + p2*cos(beta) + p3*cos(beta + gamma));
  double y = sin(alpha)*(p1 + p2*cos(beta) + p3*cos(beta + gamma));
  double z = p2*sin(beta) + p3*sin(beta + gamma);

  solution[0] = x;
  solution[1] = y;
  solution[2] = z;

  return solution;
}


bool Leg::checkReachability(const Vector3d& targetPoint)
{
  Eigen::Vector3d segm(segments[0], segments[1], segments[2]);

  if (targetPoint.lpNorm<1>() > segm.lpNorm<1>())
    return false;

  return true;
}

Eigen::Vector3d Leg::trajectoryGenerator(double time)
{
  double R = 0.025;
  double period = 3;

  Eigen::Vector3d result(3);
  result.fill(0);
  
  double x = 0;//R*sin(2*EIGEN_PI/period*time);
  double y = 0.1;
  double z = 0.0 + R*sin(2*EIGEN_PI/period*time);

  result << x, y, z;

  return result;

}

bool Leg::recieveFB(const VectorXd& feedback, int index)
{
  this->FBcoords = feedback.segment<3>(index);
  this->FBvelocities = feedback.segment<3>(index + 3);
  return true;
}

bool Leg::setCoeffs(const PID::PIDcoeffs &coeffs)
{
  for (int i = 0; i < static_cast<int>(pids.size());i++)
  {
    pids[i].setCoeffs(coeffs);
  }
  return true;
}

Eigen::VectorXd Leg::getTorques(const VectorXd& targetAngles)
{
  VectorXd torques(3);
  for(size_t i = 0; i < pids.size(); i++)
  {
    torques[i] = pids[i].getValue(targetAngles[i], FBcoords[i], FBvelocities[i]);
  }

  //TODO torque saturation

  
  return torques;
}
