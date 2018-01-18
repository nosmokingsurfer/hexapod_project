#include <mozaik/mozaik.h>


Mozaik::Mozaik()
{
 
  
}

Mozaik::~Mozaik()
{}



bool Mozaik::getAxis(const Joint& j1, const Joint& j2, Vector3d& axis1, Vector3d& axis2)
{
  axis1.fill(0);
  axis2.fill(0);
  //TODO
  return true;
}

bool Mozaik::rotateJoint(Joint& j1, const Vector3d& axis, const double angle)
{
  return true;
}

bool Mozaik::rotateAroundAxis(Joint& j1, Joint& j2, double angle)
{
  return true;
}

bool Mozaik::updateBodyGeometry(const BodyCoordinates& bodyConfig)
{
  return true;
}
