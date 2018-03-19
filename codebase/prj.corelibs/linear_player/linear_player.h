///
///@file linear_player/linear_player.h
///@authors Panchenko A.V.
///@brief Class for linear interpolation
///


#pragma once
#ifndef LINEAR_PLAYER_H
#define LINEAR_PLAYER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>


using namespace std;
using namespace Eigen;

class LinearPlayer{
public: 
  LinearPlayer();
  LinearPlayer(const VectorXd& start, const VectorXd& finish, const double T, const double startTime);
  ~LinearPlayer();
  private:
    int nDim;
    VectorXd startState;
    VectorXd finishState;
    double T;
    double startTime;
  public:
    bool reset();
    VectorXd getCurTargetState(double time);
    bool init(const VectorXd& start, const VectorXd& finish, const double T, const double startTime);
};

#endif //LINEAR_PLAYER