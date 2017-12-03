#include <step/step.h>

using namespace Eigen;

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
  double stepProgress = getStepProgress(t);

  switch(phase)
  {
    case(STAND):
      {
        result << this->a_*cos(2*EIGEN_PI*stepProgress),
                  this->h_,
                  0;
        break;
      }
    case(SWING):
      {
        result << this->a_*cos(2*EIGEN_PI*stepProgress),
                  this->h_,
                  this->b_*sin(2*EIGEN_PI*stepProgress);
        break;
      }
    case(NONE):
      {
        result.fill(0);
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

double Step::getStepProgress(const double t)
{
  double result = t - getStepNumber(t)*this->T_;
  result /= this->T_;
  return result;
}

int Step::getStepPhase(const double t)
{
  double stepProgress = this->getStepProgress(t);

  if (stepProgress <= 0.5)
  {
    return STAND;
  }
  if ((stepProgress > 0.5) && (stepProgress <= 1.0) )
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
