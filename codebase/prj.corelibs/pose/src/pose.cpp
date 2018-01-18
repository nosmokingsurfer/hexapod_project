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
            
  //std::cout << "Translation = " << endl<< Translation<double,3>(t).vector() << std::endl;                      
  //std::cout << "Rotation_Z = " << endl << AngleAxis<double>(angles[0], Vector3d::UnitZ()).matrix() << std::endl;
  //std::cout << "Rotation_X = " << endl << AngleAxis<double>(angles[1], Vector3d::UnitX()).matrix() << std::endl;
  //std::cout << "Rotation_Y = " << endl << AngleAxis<double>(angles[2], Vector3d::UnitY()).matrix() << std::endl;

  this->T = this->T.inverse();

  //std::cout << "T = " << endl << this->T.matrix() << endl;
  //std::cout << "T_Inverse = " << endl << this->T.inverse().matrix() << endl;
  //std::cout << "T*T_inverse = " << endl << (this->T*this->T.inverse()).matrix() << endl;

  
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
