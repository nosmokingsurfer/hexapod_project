///
///@file segment/segment.h
///@authors Panchenko A.V.
///@brief Class for segment
///


#pragma once
#ifndef SEGMENT_H
#define SEGMENT_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>

#include <pose/pose.h>
#include <joint/joint.h>
#include <leg/leg.h>


using namespace std;
using namespace Eigen;

class Joint;
class Leg;

class Segment{
public: 
  Segment();
  Segment(const std::string name, const Pose& pose);
  Segment(const Segment& L);
  Segment & operator=(const Segment& L);
  ~Segment();
private:
  bool initialized;
  string name; //!< name of the segment
  Pose segmentRF; //!< segment reference frame
  int fbIndex; //!< index of the segment coordinates in the FB array if any
  
public:
  bool connectLeg(Leg leg); //!< attach leg to the segment. Leg should already have a mounting pose
  bool connectJoint(Joint& joint, const bool parent); //!< attach joint to the segment. Joint should already have a mounting pose
  bool isInitialized();
  bool recieveFB(const VectorXd fb);
  string getName();
  vector<Leg> legs;//!< legs connected to the segment
  vector<Joint> joints; //!< joints connected to the segment
  
};

#endif

