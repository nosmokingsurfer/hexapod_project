#include <control/task/task.h>


Task::Task()
{}

Task::~Task()
{}

bool Task::set(const string name, const TYPE& task, const TRANSITION& transition, const Pose& a, const Pose& b, const double startTime, const double duration)
{
  this->taskType = task;
  this->transitionType = transition;
  return true;
}

bool Task::set(const string name, const TYPE& task, const TRANSITION& transition, const Pose& b, const double startTime, const double duration)
{
  cout << "Not implemented" << endl;
  return true;
}
