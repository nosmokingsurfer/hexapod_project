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
}

Robot::Robot(Body::BODY_TYPE bt)
{
  *this = Robot();

  robotBody = Body(bt);
}

Robot::~Robot()
{

}

Eigen::VectorXd Robot::getControls(double time)
{
  this->calculatedjoints = this->robotBody.getControls(time);
  
  //по рассчитанным углам считаем моменты
  VectorXd torques(controlTorques.size());
  torques.fill(0);

  for (int i = 0; i < 6; i++)
  {
    torques.segment(3*i, 3) = robotBody.segments[0].legs[i].getTorques(this->calculatedjoints.segment(3*i, 3));
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


  this->robotBody.recieveFB(signals);

  for (int i = 0; i < static_cast<int>(robotBody.segments[0].legs.size()); i++)
  {
    robotBody.segments[0].legs[i].recieveFB(feedBack);
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

  
  for(int j = 0; j < static_cast<int>(robotBody.segments.size()); j++)
  {
    for(int i = 0; i < static_cast<int>(robotBody.segments[j].legs.size()); i++)
    {
      PID::PIDcoeffs coeffs;
      coeffs.kP = parameters[0];
      coeffs.kI = parameters[1];
      coeffs.kD = parameters[2];
      robotBody.segments[j].legs[i].setCoeffs(coeffs);
    }
  }

  return true;
}

