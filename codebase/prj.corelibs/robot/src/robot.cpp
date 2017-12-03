#include <robot/robot.h>

using namespace Eigen;

Robot::Robot()
{
  //legs dimensions
  Vector3d segments(0.05, 0.05, 0.05);

  //TODO zip mounting point and mounting orientation in one class - will be able to iterate through it
  //legs mounting points
  Vector4d FLmPoint(-0.05,  0.1, 0, 0);
  Vector4d MLmPoint(-0.05,    0, 0, 0);
  Vector4d RLmPoint(-0.05, -0.1, 0, 0);
  Vector4d FRmPoint( 0.05,  0.1, 0, 0);
  Vector4d MRmPoint( 0.05,    0, 0, 0);
  Vector4d RRmPoint( 0.05, -0.1, 0, 0);
  
  
  //legs mounting orientations
  Vector3d FLmOrient(0, 0, 0);
  Vector3d MLmOrient(0, 0, 0);
  Vector3d RLmOrient(0, 0, 0);
  Vector3d FRmOrient(0, 0, 0);
  Vector3d MRmOrient(0, 0, 0);
  Vector3d RRmOrient(0, 0, 0);
  
  for(int i = 0; i < 6; i++)
  {
    robotLegs.push_back(Leg());
  }

  robotLegs[0].init(segments, FRmPoint, FRmOrient);
  robotLegs[1].init(segments, MRmPoint, MRmOrient);
  robotLegs[2].init(segments, RRmPoint, RRmOrient);
  robotLegs[3].init(segments, FLmPoint, FLmOrient);
  robotLegs[4].init(segments, MLmPoint, MLmOrient);
  robotLegs[5].init(segments, RLmPoint, RLmOrient);
}

Robot::~Robot()
{

}

Eigen::VectorXd Robot::getControls()
{
  VectorXd result(3*this->robotLegs.size());
  result.fill(0);

  for (size_t i = 0; i < 3*robotLegs.size(); i++)
  {
    result[i] = 0;
  }
  return result;
}

Eigen::VectorXd Robot::getControls(double time)
{
  VectorXd result(3*this->robotLegs.size());
  for (size_t i = 0; i < 3*robotLegs.size(); i++)
  {
    result[i] = 0.1*sin(time/(2*EIGEN_PI));
  }
  return result;
}
