#include <robot/robot.h>

using namespace Eigen;

Robot::Robot()
{
  this->controlTorques = VectorXd(18);
  this->controlTorques.fill(0);

  this->feedBack = VectorXd(48);
  this->feedBack.fill(0);

  this->calculatedjoints = VectorXd(48);
  this->calculatedjoints.fill(0);

  this->FBvelocities = VectorXd(6);
  this->FBvelocities.fill(0);

  this->FBcoords = VectorXd(6);
  this->FBcoords.fill(0);

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
  this->calculatedjoints = result;

  return result;
}

Eigen::VectorXd Robot::getControls(double time)
{
  VectorXd targetAngles(18);
  targetAngles.fill(0);

  for (int i = 0; i < 6; i++)
  {
    targetAngles[3*i + 0] = 0.1*sin(time*(2*EIGEN_PI/5));
    targetAngles[3*i + 1] = 0.1*cos(time*(2*EIGEN_PI/5));
    targetAngles[3*i + 2] = 0.2*cos(time*(2*EIGEN_PI/5));
  }

  this->calculatedjoints = targetAngles;

  VectorXd torques(controlTorques.size());

  for (int i = 0; i < 6; i++)
  {
    torques.segment(3*i, 3) = robotLegs[i].getTorques(targetAngles.segment(3*i, 3));
  }
  
  this->controlTorques = torques;
  return torques;
}

Eigen::VectorXd Robot::getCalculatedJoints()
{
  return this->calculatedjoints;
}

bool Robot::recieveFeedBack(double* inputs, int numberOfInputs)
{
  Eigen::VectorXd signals(numberOfInputs);
  signals.fill(0);

  for (int i = 0; i < numberOfInputs; i++)
  {
    signals[i] = inputs[i];
  }

  this->feedBack = signals;


  this->getFB(0);

  for (int i = 0; i < static_cast<int>(robotLegs.size()); i++)
  {
    robotLegs[i].getFB(feedBack, 12 + i*6);
  }
  return true;
}

bool Robot::recieveParameters(double* params, int numberOfParams)
{
  Eigen::VectorXd parameters(numberOfParams);
  parameters.fill(0);

  for (int i = 0; i < numberOfParams; i++)
  {
    parameters[i] = params[i];
  }

  this->parameters = parameters;

  for(int i = 0; i < static_cast<int>(robotLegs.size()); i++)
  {
    robotLegs[i].pids[0].setCoeefs(Vector3d(parameters.head(3)));
    robotLegs[i].pids[1].setCoeefs(Vector3d(parameters.head(3)));
    robotLegs[i].pids[2].setCoeefs(Vector3d(parameters.head(3)));
  }

  return true;
}

bool Robot::getFB(int index)
{
    this->FBcoords = this->feedBack.segment<6>(index);
    this->FBvelocities = this->feedBack.segment<6>(index + 6);

    return true;
}
