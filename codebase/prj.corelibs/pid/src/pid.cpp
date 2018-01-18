#include <pid/pid.h>


PID::PID():
  P(0),
  I(0),
  D(0)
{
  coeffs.kP=0;
  coeffs.kI=0;
  coeffs.kD=0;
}

PID::PID(const double kP, const double kI, const double kD)
{
  this->coeffs.kP = kP;
  this->coeffs.kI = kI;
  this->coeffs.kD = kD;
}

PID::PID(const Vector3d coeffs)
{
  this->coeffs.kP = coeffs[0];
  this->coeffs.kI = coeffs[1];
  this->coeffs.kD = coeffs[2];

  this->P = 0;
  this->I = 0;
  this->D = 0;
}

PID::PID(const PIDcoeffs& coeffs)
{
  this->coeffs = coeffs;
}

PID::~PID()
{}


bool PID::setCoeffs(const PIDcoeffs& coeffs)
{
  this->coeffs = coeffs;
  return true;
}

bool PID::setCoeffs(const double kP, const double kI, const double kD)
{
  this->coeffs.kP = kP;
  this->coeffs.kI = kI;
  this->coeffs.kD = kD;
  return true;
}

bool PID::setCoeffs(const Vector3d& coeffs)
{
  this->coeffs.kP = coeffs[0];
  this->coeffs.kI = coeffs[1];
  this->coeffs.kD = coeffs[2];
  return true;
}

bool PID::setState(const Vector3d& state)
{
  this->P = state[0];
  this->I = state[1];
  this->D = state[2];
  return true;
}

bool PID::updateState(double x, double dot_x)
{
  this->P = x;
  this->I += x;
  this->D = dot_x;

  return true;
}

double PID::getValue(double x_0, double x, double dot_x)
{
  double result = 0;

  this->updateState(x_0 - x, dot_x);

  result += coeffs.kP*P;
  result += coeffs.kI*I;
  result += coeffs.kD*D;

  return result;
}

bool PID::reset()
{
  this->coeffs.kP = 0;
  this->coeffs.kI = 0;
  this->coeffs.kD = 0;
  this->P = 0;
  this->I = 0;
  this->D = 0;
  return true;
}
