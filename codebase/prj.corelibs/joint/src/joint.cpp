#include <joint/joint.h>

Joint::Joint():
  currentState(0),
  fbIndex(0),
  ctrlIndex(0)
{
  this->pidControl.push_back(PID(0,0,0));
  this->type = JOINT_TYPE::TRANSLATION_1D;
}

Joint::Joint(const string name_, const int fbIndex_, const int ctrlIndex_, const int debugIndex_, const JOINT_TYPE type_, const Pose& parentMount, const Pose& childMount)
{
  initialized = true;

  this->name = name_;

  this->fbIndex = fbIndex_;
  this->ctrlIndex = ctrlIndex_;
  this->debugIndex = debugIndex_;

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


bool Joint::updateTransformation()
{
  switch (this->type)
  {
    case JOINT_TYPE::ROTATION_1D:
    {
      this->transformation = Pose(Vector3d(0, this->targetState[0], 0), Vector3d(0,0,0));
      break;
    }
    default: cout << "NOT IMPLEMENTED" << endl; return false;
  }
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
    this->updateTransformation();
    return true;
  }
  else
  return false;
}

Eigen::VectorXd Joint::getTargetState()
{
  return this->targetState;
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

int Joint::getDebIndex()
{
  return this->debugIndex;
}

Pose Joint::getMountInParent()
{
  return this->mountInParent;
}

Pose Joint::getMountInChild()
{
  return this->mountInChild;
}

Pose Joint::getTransformation()
{
  //todo forward or inverse transform?

  return this->transformation;
}

bool Joint::setCoeffs(const PID::PIDcoeffs& coeffs)
{
    for (int i = 0; i < static_cast<int>(pidControl.size());i++)
    {
      pidControl[i].setCoeffs(coeffs);
    }
    return true;
}
