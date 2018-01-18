///
///@file body/body.h
///@authors Panchenko A.V.
///@brief Simple body class for robot body.
///


#pragma once
#ifndef SIMPLE_BODY_H
#define SIMPLE_BODY_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <leg/leg.h>
#include <pid/pid.h>
#include <pose/pose.h>
#include <segment/segment.h>
#include <body/body.h>


#include <vector>
#include <iostream>


using namespace std;
using namespace Eigen;

class SimpleBody: public Body
{
public: 
  SimpleBody();
  ~SimpleBody();
public:

};

#endif