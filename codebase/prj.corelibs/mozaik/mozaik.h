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
private:
  vector<Joint> joints;//!< joints inside the body
  bool getAxis(const Joint& j1, const Joint& j2, Vector3d& axis1, Vector3d& axis2); //!< get axis between two joints
  bool rotateJoint(Joint& j1, const Vector3d& axis, const double angle); //!< rotates 3D spherical joint around the axis vector by the angle in radians
  bool rotateAroundAxis(Joint& j1, Joint& j2, double angle); //!< rotates the body segments around the axis going through the joints by the angle in radians
  bool updateBodyGeometry(const BodyCoordinates& bodyConfig);
};

#endif