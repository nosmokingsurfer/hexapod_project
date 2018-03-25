///
///@file control/task/task.h
///@authors Panchenko A.V.
///@brief Class for control task description
///


#pragma once
#ifndef TASK_H
#define TASK_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <map>
#include <iostream>
#include <pose/pose.h>
#include <linear_player/linear_player.h>

using namespace std;
using namespace Eigen;

class Task{
public:
  enum class TYPE
  {
    MOVE_LEG,
    MOVE_SEGMENT,
    MOVE_JOINT,
    MOVE_POSE
  };

  enum class TRANSITION
  {
    GLOBAL,
    RELATIVE
  };

public: 
  Task();
  ~Task();

  bool set(const string name, const TYPE& task, const TRANSITION& transition, const Pose& a, const Pose& b, const double startTime, const double duration);
  bool set(const string name, const TYPE& task, const TRANSITION& transition, const Pose& b, const double startTime, const double duration);
  std::map<string, LinearPlayer> tasks;

private:
  TYPE taskType;
  TRANSITION transitionType;
  Pose from;
  Pose to;
  double time;
  
  

};

#endif