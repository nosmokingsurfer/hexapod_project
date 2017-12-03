///
///@file hexapodclasses/step.h
///@authors Panchenko A.V.
///@brief Step cycle class to calculate target leg point
///

#pragma once
#ifndef STEP_H
#define STEP_H

#include <Eigen/Dense>

class Step{
public:
  Step();
  ~Step();

  enum StepPhase //TODO increase the number of states
  {
    STAND, //!< leg is in stand phase
    SWING, //!< leg is in swing motion
    FALL, //!< leg is putting on the surface
    RISE, //!< leg is lifting from the surface
    NONE //!< state is undefined
  };

  bool init(const double a, const double b, const double r, const double T);
  Eigen::Vector3d getTargetPoint(const double v, const double w, const double t);
  int getStepPhase(const double t);
  int getStepNumber(const double t);

private:
  double a_; //!< horizontal step amplitude
  double b_; //!< vertical step amplitude
  double h_; //!< step width
  double r_; //!< curvature radius at step lift off and stand
  double T_; //!< step period

  int phase_; //!< step phase
  int stepNumber_; //!< step number

private:
  Eigen::Vector3d getEigenTargetPoint(const double t); //!< get point on original step cycle before deformations
  Eigen::Vector3d adjustSpeed(Eigen::Vector3d point); //!< scale step cycle to fit the target speed
  Eigen::Vector3d adjustCurvature(Eigen::Vector3d point); //!< deform the step cycle to fit the target angular speed
  double getStepProgress(const double t); //!< get the progress in % on the step phase
};

#endif // ! STEP_H
