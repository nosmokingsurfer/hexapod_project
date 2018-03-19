#include <control/task/task.h>


Task::Task()
{}

Task::~Task()
{}

bool Task::set(const TASK& task, const TRANSITION& transition, const Pose& a, const Pose& b, const double time)
{
  this->taskType = task;
  this->transitionType = transition;
  return true;
}

bool Task::set(const TASK& task, const TRANSITION& transition, const Pose& b, const double time)
{
  cout << "Not implemented" << endl;
  return true;
}
