#include <robot/robot.h>

using namespace Eigen;

Robot::Robot()
{
  
}

Robot::Robot(Body::BODY_TYPE bt)
{
  this->robotBody = Body(bt);
  this->robotBody.printOut();

//  this->coordGraph.init(this->robotBody);

  this->feedBack = VectorXd(2*robotBody.getNDOF());
  this->feedBack.fill(0);

  this->calculatedjoints = VectorXd(robotBody.getNControls());
  this->calculatedjoints.fill(0);

}

Robot::~Robot()
{

}

Eigen::VectorXd Robot::getControls(double time)
{
  //get control angles
  this->calculatedjoints = this->robotBody.getControlAngles(time);
  

  //calculate control torques
  for (int i = 0; i < static_cast<int>(robotBody.segments.size()); i++)
  {
    for (int j = 0; j < static_cast<int>(robotBody.segments[i].legs.size());j++)
    {
      Leg& curLeg = robotBody.segments[i].legs[j];
      this->calculatedjoints.segment(curLeg.getCtrlIndex(), 3) = curLeg.getTorques();
    }
    for(int j = 0; j < static_cast<int>(robotBody.segments[i].joints.size()); j++)
    {
      Joint& curJoint = robotBody.segments[i].joints[j];
      this->calculatedjoints.segment(curJoint.getCtrlIndex(), curJoint.getDOFnumber()) = curJoint.getTorques();
    }
  }


  this->calculatedjoints.segment(40, 3) = this->getCOM();


  return this->calculatedjoints;
}


Eigen::VectorXd Robot::getControls()
{
  this->calculatedjoints.fill(0);

  //calculate control torques
  for (int i = 0; i < static_cast<int>(robotBody.segments.size()); i++)
  {
    for (int j = 0; j < static_cast<int>(robotBody.segments[i].legs.size());j++)
    {
      Leg& curLeg = robotBody.segments[i].legs[j];
      curLeg.setTargetState(Vector3d(0,0,0));
      this->calculatedjoints.segment(curLeg.getCtrlIndex(), 3) = curLeg.getTorques();
    }
    for(int j = 0; j < static_cast<int>(robotBody.segments[i].joints.size()); j++)
    {
      Joint& curJoint = robotBody.segments[i].joints[j];
      curJoint.setTargetState(VectorXd(0));
      this->calculatedjoints.segment(curJoint.getCtrlIndex(), curJoint.getDOFnumber()) = curJoint.getTorques();
    }
  }

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

  this->coordGraph.recieveFB(signals);

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

    for(int i = 0; i < static_cast<int>(robotBody.segments[j].joints.size()); i++)
    {
      PID::PIDcoeffs coeffs;
      coeffs.kP = parameters[0];
      coeffs.kI = parameters[1];
      coeffs.kD = parameters[2];
      robotBody.segments[j].joints[i].setCoeffs(coeffs);
    }
  }

  return true;
}

Eigen::Vector3d Robot::getCOM()
{
  Vector3d result = this->robotBody.getCOMcoords();

  result = this->robotBody.fbPose.T.inverse()*result;

  return result;
}

double Robot::getTotalMass()
{
  return this->robotBody.getTotalMass();
}
