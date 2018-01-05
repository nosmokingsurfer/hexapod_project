#include <joint/joint.h>

Joint::Joint():
  currentState(0)
{
  this->pidControl.push_back(PID(0,0,0));
  this->type = JOINT_TYPE::TRANSLATION_1D;
}

Joint::~Joint()
{

}

bool Joint::init(const string name, const int fbIndex, const JOINT_TYPE type, const PID::PIDcoeffs coeffs, Segment *parent, Segment *child)
{
  initialized = true;

  this->name = name;

  this->fbIndex = fbIndex;

  this->type = type;
  

  switch(type)
  {
    case ROTATION_1D:    currentState.resize(1); break;
    case ROTATION_2D:    currentState.resize(2); break;
    case ROTATION_3D:    currentState.resize(3); break;
    case TRANSLATION_1D: currentState.resize(1); break;
    case TRANSLATION_2D: currentState.resize(2); break;
    case TRANSLATION_3D: currentState.resize(3); break;
  }

  currentState.fill(0);

  currentVelocities.resize(currentState.size());
  currentVelocities.fill(0);

  targetState.resize(currentState.size());
  targetState.fill(0);

  pidControl.resize(currentState.size());
  for (int i = 0; i < static_cast<int>(pidControl.size()); i++)
  {
    initialized &= pidControl[i].setCoeffs(coeffs);
  }

  this->parent = parent;    
  this->child = child;

  return initialized;
}

bool Joint::recieveFB()
{
  return true;
}

VectorXd Joint::getState()
{
  return this->currentState;
}

bool Joint::setState(VectorXd state)
{
  if(state.size() == this->currentState.size())
  {
    this->currentState = state;
    return true;
  }
  else
    return false;
}

bool Joint::setTargetState(VectorXd targetState)
{
  if(targetState.size() == this->targetState.size())
  {
    this->targetState = targetState;
    return true;
  }
  else
  return false;
}

VectorXd Joint::getTorques()
{
  VectorXd result(currentState.size());
  result.fill(0);

  for (int i = 0; i < currentState.size(); i++)
  {
    result[i] = pidControl[i].getValue(targetState[i], currentState[i], currentVelocities[i]);
  }

  return result;
}

int Joint::getDOFnumber()
{
  return static_cast<int>(this->currentState.size());
}
