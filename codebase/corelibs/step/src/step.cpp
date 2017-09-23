#include <step/step.h>

Step::Step()
{

}

Step::~Step()
{

}

bool Step::init(const double a, const double b, const double r, const double T)
{
  this->a_ = a;
  this->b_ = b;
  this->r_ = r;
  this->T_ = T;

  return true;
}

Eigen::Vector3d Step::getTargetPoint(const double v, const double w, const double t)
{
  Eigen::Vector3d result;
  result.fill(0);

  result = getEigenTargetPoint(t); //get point at default step cycle curvature
  result = adjustSpeed(result); //scale the step cycle to fit the target speed during stand phase
  result = adjustCurvature(result); //bend the stand phase to fit the desired omega

  return result;
}

Eigen::Vector3d Step::getEigenTargetPoint(const double t)
{
  Eigen::Vector3d result;
  result.fill(0);

  int phase = getStepPhase(t);

  switch(phase)
  {
    case(STAND):
      {
        break;
      }
    case(SWING):
      {
        break;
      }
  }
  return result;
}

Eigen::Vector3d Step::adjustSpeed(Eigen::Vector3d point)
{
  return point;
}

Eigen::Vector3d Step::adjustCurvature(Eigen::Vector3d point)
{
  return point;
}

int Step::getStepPhase(const double t)
{
  double stepProgress = t - getStepNumber(t)*this->T_;
  stepProgress /= this->T_;
  stepProgress *= 100;

  if (stepProgress <= 50)
  {
    return STAND;
  }
  if ((stepProgress > 50) && (stepProgress <= 100) )
  {
    return SWING;
  }

  return NONE;
}

int Step::getStepNumber(const double t)
{
  int result;
  result = int(t / this->T_);
  return result;
}
