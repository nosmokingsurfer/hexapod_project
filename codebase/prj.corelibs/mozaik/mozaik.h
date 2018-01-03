///
///@file dummy/dummy.h
///@authors Panchenko A.V.
///@brief Class to work with mozaik type of Body
/// Has specific methods to work with 2 by 3 mozaik body
///


#pragma once
#ifndef MOZAIK_H
#define MOZAIK_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>

#include <body/body.h>


using namespace std;
using namespace Eigen;

class Mozaik : public Body
{
public: 
  Mozaik();
  ~Mozaik();
};

#endif