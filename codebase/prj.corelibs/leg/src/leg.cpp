//#include "stdafx.h"
#include <leg/leg.h>


Leg::Leg()
{
  this->segments.clear();
  this->mounting.fill(0);
}


Leg::~Leg()
{

}

bool Leg::init(const vector<double>& segments, const Eigen::Vector4d& mountingPoint, const Eigen::Vector3d& mountingAngles)
{
  for (size_t i = 0; i < segments.size(); i++)
  {
    this->segments.push_back(segments[i]);
  }

  this->mounting.col(3) = Vector4d(mountingPoint);

  Matrix3d temp;

  temp = AngleAxisd(mountingAngles[0], Vector3d::UnitX())
        *AngleAxisd(mountingAngles[1], Vector3d::UnitY())
        *AngleAxisd(mountingAngles[2], Vector3d::UnitZ());

  this->mounting.topLeftCorner(3, 3) = temp;
 

  Eigen::Transform<double, 3, Eigen::TransformTraits::Affine> test;
  test.matrix().topLeftCorner(3,3) = temp;
  test.matrix().col(3) = mountingPoint;
  
  //PID controller for all joints
  pids.push_back(PID(Vector3d(0,0,0)));
  pids.push_back(PID(Vector3d(0,0,0)));
  pids.push_back(PID(Vector3d(0,0,0)));


  return true;
}

bool Leg::init(const Eigen::Vector3d& segments, const Vector4d& mountingPoint, const Vector3d& mountingAngles)
{
  std::vector<double> segms;
  segms.push_back(segments[0]);
  segms.push_back(segments[1]);
  segms.push_back(segments[2]);

  return this->init(segms,mountingPoint,mountingAngles);
}

void Leg::inverseKinematics(Eigen::Vector3d& targetPoint, Eigen::Vector3d& solution)
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
    return;

  alpha = atan2(y, x);
  double x_cap = sqrt(x*x + y*y);
  double l = sqrt(z*z + (x_cap - p1)*(x_cap - p1));

  double psi1 = atan2(z, x_cap - p1);
  double psi2 = acos((p2*p2 + l*l - p3*p3)/(2*p2*l));  

  beta = psi1 + psi2;

  gamma = acos((l*l - p2*p2 - p3*p3)/(-2*p2*p3)) - EIGEN_PI;

  solution[0] = alpha;
  solution[1] = beta;
  solution[2] = gamma;
  
}


void Leg::forwardKinematics(Eigen::Vector3d& targetAngles, Eigen::Vector3d& solution)
{
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
}


bool Leg::checkReachability(Vector3d& targetPoint)
{
  Eigen::Vector3d segm(segments[0], segments[1], segments[2]);

  if (targetPoint.lpNorm<1>() > segm.lpNorm<1>())
    return false;

  return true;
}

Eigen::Vector3d Leg::trajectoryGenerator(double time)
{
  double R = 0.3;
  double period = 3;

  Eigen::Vector3d result(3);
  result.fill(0);
  
  double x = 3;
  double y = R*sin(2*EIGEN_PI/period*time);
  double z = R*cos(2*EIGEN_PI/period*time);

  result << x, y, z;

  return result;

}

bool Leg::getFB(VectorXd& feedback, int index)
{
  this->FBcoords = feedback.segment<3>(index);
  this->FBvelocities = feedback.segment<3>(index + 3);
  return true;
}

Eigen::VectorXd Leg::getTorques(const VectorXd& targetAngles)
{
  VectorXd torques(3);
  for(size_t i = 0; i < pids.size(); i++)
  {
    torques[i] = pids[i].getValue(targetAngles[i], FBcoords[i], FBvelocities[i]);
  }
  
  return torques;
}
