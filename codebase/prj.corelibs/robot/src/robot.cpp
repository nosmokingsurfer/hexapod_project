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
}

Robot::~Robot()
{

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

  

  for (int i = 0; i < 6; i++)
  {
    targetAngles.segment(3*i,3) = robotBody.legs[i].inverseKinematics(robotBody.legs[i].trajectoryGenerator(time));
  }

  this->calculatedjoints = targetAngles;


  VectorXd torques(controlTorques.size());

  for (int i = 0; i < 6; i++)
  {
    torques.segment(3*i, 3) = robotBody.legs[i].getTorques(targetAngles.segment(3*i, 3));
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


  this->robotBody.getFB(signals);

  for (int i = 0; i < static_cast<int>(robotBody.legs.size()); i++)
  {
    robotBody.legs[i].getFB(feedBack, 12 + i*6);
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

  for(int i = 0; i < static_cast<int>(robotBody.legs.size()); i++)
  {
    robotBody.legs[i].pids[0].setCoeefs(Vector3d(parameters.head(3)));
    robotBody.legs[i].pids[1].setCoeefs(Vector3d(parameters.head(3)));
    robotBody.legs[i].pids[2].setCoeefs(Vector3d(parameters.head(3)));
  }

  return true;
}

