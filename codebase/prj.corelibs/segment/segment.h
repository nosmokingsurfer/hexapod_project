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

class Segment{
public: 
  Segment();
  Segment(const std::string name, const Pose& pose);
  ~Segment();
  bool init(const vector<Leg> &legs);
public:
  string name; //!< name of the segment
  Pose segmentRF; //!< segment reference frame
  int fbIndex; //!< index of the segment coordinates in the FB array if any
  vector<Leg> legs;//!< legs connected to the segment
  vector<Joint> joints; //!< joints connected to the segment
  
};

#endif

