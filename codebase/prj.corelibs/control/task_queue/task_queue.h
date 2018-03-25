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
#include <linear_player/linear_player.h>
#include <../thirdparty/jsoncpp_headers/json/json.h>

#include <vector>
#include <map>
#include <queue>
#include <iostream>


using namespace std;
using namespace Eigen;

class TaskQueue{
public: 
  TaskQueue();
  ~TaskQueue();

  bool init(const std::string scriptname);

  bool addTask(const Task&);
  bool readCommand(const double time);
  VectorXd getControls(const std::string objectName, const double time);

  static Json::Value setLeg(const Vector3d& from, const Vector3d& to, const double duration, const double start_time);
  std::map<std::string, LinearPlayer> controlSignals;

private:
  
  Json::Value controlScript;
};

#endif