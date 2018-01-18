///
///@file articulated/articulated.h
///@authors Panchenko A.V.
///@brief Class for articulated body
///


#pragma once
#ifndef ARTICULATED_H
#define ARTICULATED_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>

#include <body/body.h>


using namespace std;
using namespace Eigen;

class Articulated: public Body
{
public: 
  Articulated();
  ~Articulated();
};

#endif