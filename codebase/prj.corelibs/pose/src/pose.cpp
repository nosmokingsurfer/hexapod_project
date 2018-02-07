#include <pose/pose.h>

using namespace Eigen;
using namespace std;



Pose::Pose()
{
  (*this) = Pose(Vector3d(0,0,0), Vector3d(0,0,0));  
}

Pose::Pose(Vector3d angles)
{
  (*this) = Pose(angles, Vector3d(0,0,0));
}


Pose::Pose(Vector3d angles, Vector3d t)
{
  
  this->T = Translation<double,3>(t)
            *AngleAxis<double>(angles[0], Vector3d::UnitZ())
            *AngleAxis<double>(angles[1], Vector3d::UnitX())
            *AngleAxis<double>(angles[2], Vector3d::UnitY());

  this->T = this->T.inverse();  
}

Pose::Pose(const Transform<double, 3, Affine>& _T)
{
  this->T = _T;
}

Pose::~Pose()
{}

Eigen::MatrixXd Pose::getRotation()
{
  return this->T.rotation();
}

Eigen::Vector3d Pose::getTranslation()
{
  return this->T.translation();
}

bool Pose::setPosition(Eigen::Vector3d position)
{
  this->T.translation() = position;
  return true;
}


Pose Pose::getRotationAroundAxis(Vector3d axis, double angle)
{
  Pose result;

  result.T = AngleAxis<double>(angle, axis);
  result.T = result.T.inverse();

  return result;
}

Pose Pose::operator*(Pose pose)
{
  return Pose(this->T*pose.T);
}

Pose Pose::inverse()
{
  return Pose(this->T.inverse());
}
