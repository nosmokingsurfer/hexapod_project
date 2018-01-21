#include <joint/joint.h>

Joint::Joint():
  currentState(0),
  fbIndex(0),
  ctrlIndex(0)
{
  this->pidControl.push_back(PID(0,0,0));
  this->type = JOINT_TYPE::TRANSLATION_1D;
}

Joint::Joint(const string name_, const int fbIndex_, const int ctrlIndex_, const JOINT_TYPE type_, const Pose& parentMount, const Pose& childMount)
{
  initialized = true;

  this->name = name_;

  this->fbIndex = fbIndex_;
  this->ctrlIndex = ctrlIndex_;

  this->type = type_;

  this->mountInChild = childMount;
  this->mountInParent = parentMount;


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

bool Joint::recieveFB(const VectorXd& fb)
{
  int N = getDOFnumber();

  this->currentState = fb.segment(this->fbIndex, N);
  this->currentVelocities = fb.segment(this->fbIndex + N, N);

  return true;
}

VectorXd Joint::getState()
{
  return this->currentState;
}

void Joint::updateTransformMatrix(VectorXd state)
{
  cout << "NOT IMPLEMENTED" << endl;
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


bool Joint::setMountInParent(const Pose& mountingPose)
{
  this->mountInParent = mountingPose;
  return true;
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


bool Joint::setMountInChild(const Pose& mountingPose)
{
  this->mountInChild = mountingPose;
  return true;
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


int Joint::getCtrlIndex()
{
  return this->ctrlIndex;
}
