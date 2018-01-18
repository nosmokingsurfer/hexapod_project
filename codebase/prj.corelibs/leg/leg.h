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
  Leg(const Leg& L);
  Leg & operator=(const Leg&);
  Leg(const std::string name, const int fbIndex, const Vector3d& segments, const Pose& mountingPose);
  ~Leg();

 //bool init(const std::string name, const Vector3d& segments, const Pose& mountingPose); //TODO выилиить такую инициализацию - сделать set/get методы
public:
  vector<double> segments;
  Eigen::Vector3d inverseKinematics(const Vector3d& targetPoint);
  Eigen::Vector3d forwardKinematics(const Vector3d& targetAngles);
  bool checkReachability(const Vector3d& targetPoint);
  bool recieveFB(const VectorXd& feedback);
  bool setCoeffs(const PID::PIDcoeffs &coeffs);
  bool setFBindex(const int index);


  Eigen::Vector3d trajectoryGenerator(double time);
  Eigen::VectorXd getTorques(const VectorXd& targetAngles);

  std::vector<PID> pids; // all PID controllers of the leg

  Pose pose;//leg mounting pose

  bool setParentSegment(Segment&);

  string getName();
  string getParentName();

private:
  int fbIndex; //TODO: протащить в конструктор
  std::string legName; //!< leg name
  Vector3d FBcoords; //!< leg state - array of joint angles
  Vector3d FBvelocities; //!< leg coordinate velocities
  Vector3d targetState; //!< current leg target position

  Segment* parent;//!< the segment leg connected to
};

#endif