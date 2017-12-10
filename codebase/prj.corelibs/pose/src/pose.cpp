#include <pose/pose.h>

using namespace Eigen;


Pose::Pose()
{
  this->R = MatrixXd::Identity(3,3);

  this->t = Vector3d(3);
  t.fill(0);
}

Pose::Pose(Vector3d t)
{
  this->t = t;
  this->R  = MatrixXd::Identity(3,3);
}

Pose::Pose(Vector3d t, MatrixXd R)
{
  this->t = t;
  this->R = R;
}

Pose::~Pose()
{}

Eigen::MatrixXd Pose::getOrientationMatrix()
{
  return this->R;
}

Eigen::Vector3d Pose::getPosition()
{
  return this->t;
}

bool Pose::setOrientationMatrix(Eigen::MatrixXd orientationMatrix)
{
  this->R = orientationMatrix;
  return true;
}

bool Pose::setPosition(Eigen::Vector3d position)
{
  this->t = position;
  return true;
}

Eigen::MatrixXd Pose::getCoordsInPoseReferenceFrame(MatrixXd externalCoords)
{
  return this->R*(externalCoords - this->t);
}

Eigen::MatrixXd Pose::getCoordsOutsidePoseReferenceFrame(MatrixXd internalCoords)
{
  return this->t + this->R.inverse()*internalCoords;
}

Pose Pose::inverse()
{
  this->t = -this->R*t;
  this->R = R.inverse();
  return Pose(this->t, this->R);
}
