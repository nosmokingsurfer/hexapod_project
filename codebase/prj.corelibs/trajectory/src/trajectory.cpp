#include <Trajectory/Trajectory.h>


Trajectory::Trajectory()
{}

Trajectory::~Trajectory()
{}

bool Trajectory::init(const vector<Pose> &controlPoints)
{
  this->controlPoints = controlPoints;
  return true;
}
