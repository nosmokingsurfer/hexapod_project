///
///@file dummy/dummy.h
///@authors Panchenko A.V.
///@brief Dummy class for Core  libs
///


#pragma once
#ifndef DUMMY_H
#define DUMMY_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>


using namespace std;
using namespace Eigen;

class Dummy{
public: 
  Dummy();
  ~Dummy();
};

#endif