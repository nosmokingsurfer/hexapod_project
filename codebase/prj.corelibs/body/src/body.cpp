#include <body/body.h>


Body::Body()
{
  this->fbIndex = 0;
  this->segments.clear();
}

Body::~Body()
{}

bool Body::recieveFB(VectorXd& feedback)
{
  this->FBcoords = feedback.segment<6>(this->fbIndex);
  this->FBvelocities = feedback.segment<6>(this->fbIndex + 6);

  this->fbPose = Pose(FBcoords.tail(3), FBcoords.head(3));

  return true;
}

Pose Body::getTargetPose(double time)
{
  Vector3d xyz;

   xyz <<
    0.005*sin(2*EIGEN_PI/3*time),
    0.005*sin(2*EIGEN_PI/3*time),
    0.025 + 0.005*sin(2*EIGEN_PI/4*time);

   Vector3d YawPitchRoll;
    YawPitchRoll << 
      0.05*sin(2*EIGEN_PI/2*time),
      0.05*sin(2*EIGEN_PI/3*time),
      0.05*sin(2*EIGEN_PI/4*time);

  return Pose(YawPitchRoll, xyz);
}


void Body::printOut()
{
  for (int i = 0; i < this->segments.size(); i++)
  {
    cout << this->segments[i].getName() << endl;
    for (int j = 0; j < this->segments[i].legs.size(); j++)
    {
      cout << "\t" << this->segments[i].legs[j].getName() << endl;
      cout << "\tparent: " << this->segments[i].legs[j].getParentName() << endl;
    } 
    for(int j = 0; j < this->segments[i].joints.size(); j++)
    {
      cout << this->segments[i].joints[j].getName() << endl;
      cout << "\tparent: " << this->segments[i].joints[j].getParentName() << endl;
      cout << "\tchild: " << this->segments[i].joints[j].getChildName() << endl;
    }
  }


}
