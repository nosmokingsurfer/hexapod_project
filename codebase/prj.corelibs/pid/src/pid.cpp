#include <pid/pid.h>


PID::PID():
  kP(0),
  kI(0),
  kD(0),
  P(0),
  I(0),
  D(0)
{
}

PID::PID(double kP, double kI, double kD)
{
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
}

PID::PID(Vector3d coeffs)
{
  this->kP = coeffs[0];
  this->kI = coeffs[1];
  this->kD = coeffs[2];

  this->P = 0;
  this->I = 0;
  this->D = 0;
}

PID::~PID()
{}


bool PID::setCoeefs(double kP, double kI, double kD)
{
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  return true;
}

bool PID::setCoeefs(Vector3d& coeffs)
{
  this->kP = coeffs[0];
  this->kI = coeffs[1];
  this->kD = coeffs[2];
  return true;
}

bool PID::setState(Vector3d& state)
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

  result += kP*P;
  result += kI*I;
  result += kD*D;

  return result;
}

bool PID::reset()
{
  this->kP = 0;
  this->kI = 0;
  this->kD = 0;
  this->P = 0;
  this->I = 0;
  this->D = 0;
  return true;
}
