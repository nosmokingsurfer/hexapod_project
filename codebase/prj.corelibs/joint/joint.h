///
///@file joint/joint.h
///@authors Panchenko A.V.
///@brief Class for joint
///


#pragma once
#ifndef JOINT_H
#define JOINT_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>

#include <pid/pid.h>
#include <segment/segment.h>


using namespace std;
using namespace Eigen;

class Segment;

class Joint{
  public:
    enum class JOINT_TYPE
    { 
      ROTATION_1D, 
      ROTATION_2D, 
      ROTATION_3D, 
      TRANSLATION_1D,
      TRANSLATION_2D,
      TRANSLATION_3D
    };
public: 
  Joint();
  Joint(const string name, const int fbIndex, const int ctrlIndex, const JOINT_TYPE type, const Pose& parentMount, const Pose& childMount);
  ~Joint();
  
  bool recieveFB(const VectorXd& feedback); //!< receive FB from UM model

private:
  bool initialized;//!< initialization flag

  string name;//!< name of element

  vector<PID> pidControl;//!< PID controllers to calculate torques //TODO make abstract Controller class?

  int fbIndex;//!< index of joint coordinates inside the FB array
  int ctrlIndex; //!< index of joint torques inside the control signals array

  JOINT_TYPE type;//!< joint type

  VectorXd currentState; //!< current joint state - to be used inside the PID controller
  VectorXd currentVelocities; //!< current joint velocities
  VectorXd targetState; //!< target joint state - to be used inside the PID controller

  Segment* parent; //!< the parent segment to which the joint is connected to
  Segment* child; //!< the child segment to which the joint is connected to

  Pose transformation; //!< what transformation joint performs
  Pose mountInParent; //!< mounting pose in parent segment
  Pose mountInChild; //!< mounting pose in child segment

  void updateTransformMatrix(VectorXd state);

private:
  bool updateTransformation(); //!< updates the transformation Pose matrix with relevance to the current state
public:
  bool setName(string name);
  bool setFBindex(const int index);
  bool setState(VectorXd state); //!< set current state of the joint
  bool setTargetState(VectorXd targetState); //!< set target state of the joint

  bool isInitialized(); //!< if the joint initialized or not
  string getName(); //!< returns the name of the segment
  string getParentName();
  string getChildName();

  bool setParentSegment(Segment& seg);
  bool setMountInParent(const Pose& mountingPose);

  bool setChildSegment(Segment& seg);
  bool setMountInChild(const Pose& mountingPose);

  VectorXd getState(); //!< get current joint state of the joint
  VectorXd getTorques(); //!< get calculated torques from the joint control system
  int getDOFnumber(); //!< returns number of degrees of freedom
  int getCtrlIndex();

};

#endif