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
#include <iostream>
#include <pose/pose.h>

using namespace std;
using namespace Eigen;

class Task{
public:
  enum class TASK
  {
    MOVE_LEG,
    MOVE_PAIRED_LEGS,
    MOVE_MIRRORED_LEGS,
    MOVE_BODY,
  };

  enum class TRANSITION
  {
    GLOBAL,
    RELATIVE
  };

public: 
  Task();
  ~Task();

  bool set(const TASK& task, const TRANSITION& transition, const Pose& a, const Pose& b, const double time);
  bool set(const TASK& task, const TRANSITION& transition, const Pose& b, const double time);

private:
  TASK taskType;
  TRANSITION transitionType;
  Pose from;
  Pose to;
  double time;
  
};

#endif