#include <linear_player/linear_player.h>


LinearPlayer::LinearPlayer()
{
  this->startState = VectorXd(1);
  startState << 0;

  this->finishState = VectorXd(1);
  finishState << 0;

  this->nDim = 1;
  this->T = 1;
  this->startTime = 0;
}

LinearPlayer::LinearPlayer(const VectorXd& start, const VectorXd& finish, const double T, const double startTime)
{
  this->init(start,finish,T,startTime);
}

LinearPlayer::~LinearPlayer()
{}



VectorXd LinearPlayer::getCurTargetState(double time)
{
  VectorXd result = this->startState;

  double lambda = 0;
  double acceleration_time = 0.3; // 3 seconds to accelerate
  
  if (time <= startTime)
  {
    lambda = 0;
  }

  //acceleration step
  else if((time > startTime) && (time < startTime + acceleration_time))
  {
    lambda = pow(time - startTime,2)/(2*(this->T - acceleration_time)*acceleration_time);
  }

  //constant speed motion
  else if ((time >= startTime + acceleration_time) && (time <= startTime + T - acceleration_time))
  {
    lambda = pow(acceleration_time, 2)/(2*(this->T - acceleration_time)*acceleration_time) + (time - startTime - acceleration_time)/(this->T - acceleration_time);
  }

  //decceleration step
  else if((time > startTime + T - acceleration_time)&&(time < startTime + T))
  {
    lambda = ((time - startTime - (this->T - acceleration_time))*(this->T - time + startTime + acceleration_time))/(2*(acceleration_time*(this->T - acceleration_time)));
    lambda += pow(acceleration_time, 2)/(2*(this->T - acceleration_time)*acceleration_time) + (startTime + T - acceleration_time - startTime - acceleration_time)/(this->T - acceleration_time);
  }
  
  else if (time >= startTime + T)
  {
    lambda = 1;
  }

  cout << lambda << endl;

  return ((1-lambda)*startState + lambda*finishState);
}

bool LinearPlayer::init(const VectorXd& start, const VectorXd& finish, const double T, const double startTime)
{
  if((start.size() == finish.size()) && (T > 0))
  {
    this->startState = start;
    this->finishState = finish;
    this->nDim = static_cast<int>(startState.size());
    this->T = T;
    this->startTime = startTime;
    return true;
  }
  else
  {
    cout << "ERROR!" << endl;
    return false;
  }
}
