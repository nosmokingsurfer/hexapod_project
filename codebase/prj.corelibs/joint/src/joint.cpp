#include <joint/joint.h>

Joint::Joint():
  currentState(0)
{
  this->pidControl.push_back(PID(0,0,0));
  this->type = JOINT_TYPE::TRANSLATION_1D; //prun: either use enum class or don't use enum name::as a namespace
}

Joint::Joint(const string name, const int fbIndex, const JOINT_TYPE type)
{
  initialized = true;

  this->name = name;

  this->fbIndex = fbIndex;

  this->type = type;


  switch(type)
  {
  case JOINT_TYPE::ROTATION_1D:    currentState.resize(1); break;
  case JOINT_TYPE::ROTATION_2D:    currentState.resize(2); break;
  case JOINT_TYPE::ROTATION_3D:    currentState.resize(3); break;
  case JOINT_TYPE::TRANSLATION_1D: currentState.resize(1); break;
  case JOINT_TYPE::TRANSLATION_2D: currentState.resize(2); break;
  case JOINT_TYPE::TRANSLATION_3D: currentState.resize(3); break;
  }

  currentState.fill(0);

  currentVelocities.resize(currentState.size());
  currentVelocities.fill(0);

  targetState.resize(currentState.size());
  targetState.fill(0);

  pidControl.resize(currentState.size());
  for (int i = 0; i < static_cast<int>(pidControl.size()); i++)
  {
    initialized &= pidControl[i].setCoeffs(PID::PIDcoeffs());
  }
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
    case JOINT_TYPE::ROTATION_1D:    currentState.resize(1); break;
    case JOINT_TYPE::ROTATION_2D:    currentState.resize(2); break;
    case JOINT_TYPE::ROTATION_3D:    currentState.resize(3); break;
    case JOINT_TYPE::TRANSLATION_1D: currentState.resize(1); break;
    case JOINT_TYPE::TRANSLATION_2D: currentState.resize(2); break;
    case JOINT_TYPE::TRANSLATION_3D: currentState.resize(3); break;
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
  cout << "NOT IMPLEMENTED" << endl;
  return true;
}

VectorXd Joint::getState()
{
  return this->currentState;
}

bool Joint::updateTransformation()
{
  cout << "NOT IMPLEMENTED" << endl;
  return true;
}

bool Joint::setName(string name)
{
  if (name.size() > 0)
  {
    this->name = name;
    return true;
  }
  else
    return false;
}

bool Joint::setFBindex(const int index)
{
  if (index >= 0)
  {
    this->fbIndex = index;
    return true;
  }
  else
    return false;
}

bool Joint::isInitialized()
{
  return this->initialized;
}

std::string Joint::getName()
{
  return this->name;
}

std::string Joint::getParentName()
{
  return this->parent->getName();
}

std::string Joint::getChildName()
{
  return this->child->getName();
}

bool Joint::setParentSegment(Segment& seg)
{
  if(seg.isInitialized())
  {
    this->parent = &seg;
    return true;
  }
  else
    return false;
}


bool Joint::setChildSegment(Segment& seg)
{
  if(seg.isInitialized())
  {
    this->child = &seg;
    return true;
  }
  else
    return false;
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


