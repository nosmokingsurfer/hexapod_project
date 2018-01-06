#include <trajectory/trajectory.h> //prun: unix is case-sensitive


Trajectory::Trajectory()
{}

Trajectory::~Trajectory()
{}

bool Trajectory::init(const vector<Pose> &controlPoints)
{
  this->controlPoints = controlPoints;
  return true;
}
