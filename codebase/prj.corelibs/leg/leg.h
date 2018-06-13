///
///@file hexapodclasses/leg.h
///@authors Panchenko A.V.
///@brief Leg class for inverse kinematics and etc
///


#pragma once
#ifndef LEG_H
#define LEG_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>

#include <pose/pose.h>
#include <segment/segment.h>
#include <pid/pid.h>

using namespace std;
using namespace Eigen;

class Segment;

class Leg{
public: 
  Leg();
  Leg(const std::string name, const int fbIndex, const int ctrlIndex, const int debIndex, const Vector3d& segments, const Pose& mountingPose);
  Leg(const std::string name, const int fbIndex, const int ctrlIndex, const int debIndex, const Vector3d& segments, const Pose& mountingPose, const vector<double>& masses);
  ~Leg();
public:
  vector<double> segments;
  Eigen::Vector3d inverseKinematics(const Vector3d& targetPoint);
  Eigen::Vector3d forwardKinematics(const Vector3d& targetAngles);

  Eigen::MatrixXd jacobian(const Vector3d& jointAngles);
  Eigen::MatrixXd jacobian_inverted(const Vector3d& jointAngles);
  Eigen::Vector3d numericalSolve(const Vector3d& targetPoint);
  Eigen::Vector3d numericalSolve(const Vector3d& targetPoint, const Vector3d& initial);

  bool checkReachability(const Vector3d& targetPoint); //!< check if the leg can reach the target point
  bool recieveFB(const VectorXd& feedback); //!< copy feedback signals in FBcoords and FBvelocities from common feedback vector
  bool setCoeffs(const PID::PIDcoeffs &coeffs);//!< set coefficients of the PID controllers inside all leg joints
  bool setFBindex(const int index); //!< set the offset inside the common feedback vector to collect proper corresponding feedback signals
  int getCtrlIndex(); //!< returns control signal index inside common control vector of the system
  int getDebIndex(); //!< returns index in debug part of common output vector of the system


  Eigen::Vector3d trajectoryGenerator(double time); //!< generates trajectory for the leg with reference to time variable
  Eigen::VectorXd getTorques(); //!< calculates control torques for given targetPoint and current FBCoords

  std::vector<PID> pids; //!< all PID controllers of the leg

  Pose pose;//!< leg mounting pose

  bool setParentSegment(Segment&); //!< set pointer to parent segment

  string getName(); //!< returns name of the leg
  string getParentName(); //!< returns the parent segment name
  bool setTargetState(const Vector3d& targetstate); //!< sets target point for leg
  Vector3d getTargetState(); //!< returns targetPoint value



  Vector3d getCOMcoords(); //!< returns center of mass coordinates in leg's reference frame
  double getTotalMass(); //!< returns total mass of the leg

private:
  int fbIndex; // todo move to the basic class
  int ctrlIndex; //todo move to the basic class
  int debIndex; //!< debug index of the leg
  std::string legName; //!< leg name // todo move to the basic class as name
  Vector3d FBcoords; //!< leg state - array of joint angles
  Vector3d FBvelocities; //!< leg coordinate velocities
  Vector3d targetState; //!< current leg target position

  Segment* parent;//!< the segment leg connected to

  std::vector<double> masses; //!< masses for all segments of the leg
  double totalMass; //!< total mass of the leg

};

#endif