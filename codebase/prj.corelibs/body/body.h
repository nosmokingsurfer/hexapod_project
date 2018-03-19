///
///@file body/body.h
///@authors Panchenko A.V.
///@brief Body class for robot body
///


#pragma once
#ifndef BODY_H
#define BODY_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <leg/leg.h>
#include <pid/pid.h>
#include <pose/pose.h>
#include <segment/segment.h>
#include <linear_player/linear_player.h>


#include <vector>
#include <iostream>


using namespace std;
using namespace Eigen;

class Body{
public: 
  enum class BODY_TYPE{
    SIMPLE,
    ARTICULATED,
    MOZAIK
  };

  Body();
  Body(BODY_TYPE bt);
  ~Body();
private:
  void initSimpleBody();
  void initArticulatedBody();
  void initMozaikBody();

  VectorXd getSimpleBodyControlAngles(double time);
  VectorXd getArticulatedBodyControlAngles(double time);
  VectorXd getMozaikBodyControlAngles(double time);

  int nDOF; //!< number of degrees of freedom from FB
  int nControls; //!< number of control torques
  
  struct BodyCoordinates
  {
    double order;
    double normal;
    double longitudal;
    double lateral_forward;
    double lateral_rear;
    double leaf_FL;
    double leaf_FR;
    double leaf_RL;
    double leaf_RR;
  };

  bool getAxis(const Joint& j1, const Joint& j2, Vector3d& axis1, Vector3d& axis2); //!< get axis between two joints
  bool rotateJoint(Joint& j1, const Vector3d& axis, const double angle); //!< rotates 3D spherical joint around the axis vector by the angle in radians
  bool rotateAroundAxis(Joint& j1, Joint& j2, double angle); //!< rotates the body segments around the axis going through the joints by the angle in radians
  bool updateBodyGeometry(const BodyCoordinates& bodyConfig);

public:
  vector<Segment> segments;
  Pose fbPose; //!< feedback pose of the robot's body
  Pose tarPose; //!< target pose of the robot's body

  VectorXd FBcoords;
  VectorXd FBvelocities;
  
  bool recieveFB(const VectorXd& feedback);// get feedback from UM
  Pose getTargetPose(double time); // get target position of body in global RF

  VectorXd getControlAngles(double time);

  void printOut();//!< print all the element of the mozaik body into the console

  BODY_TYPE body_type; //!< kinematics of the body

  int getNDOF(); //!< returns number of DOF
  int getNControls(); //!< returns number of controls
};


#endif