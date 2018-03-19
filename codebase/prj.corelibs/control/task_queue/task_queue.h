///
///@file dummy/dummy.h
///@authors Panchenko A.V.
///@brief Dummy class for Core  libs
///


#pragma once
#ifndef TASK_QUEUE_H
#define TASK_QUEUE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <control/task/task.h>

#include <vector>
#include <queue>
#include <iostream>


using namespace std;
using namespace Eigen;

class TaskQueue{
public: 
  TaskQueue();
  ~TaskQueue();

  std::queue<Task> tasks;

  bool addTask(const Task&);
  
};

#endif