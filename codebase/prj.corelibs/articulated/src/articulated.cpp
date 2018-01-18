#include <articulated/articulated.h>


Articulated::Articulated()
{
  ///the body initialization order as follows:
  /// 1) all segments - w/o legs and joints
  /// 2) all legs - connect to segments: define parents and childrens in pointers
  /// 3) all joints between segments - will be able to define all parents and childrens

  //////////////////////////////////////////////////////////////////////////
  // Defining all segments of the body
  //
  //
  //       RR - rare  - RL
  //              |   
  //              ^   
  //            j_2_3 
  //              ^   
  //              |   
  //       MR -  mid  - ML
  //              |   
  //              ^   
  //            j_1_2 
  //              ^   
  //              |   
  //       FR - front - FL
  //
  //
  //////////////////////////////////////////////////////////////////////////
  this->segments.clear();
  this->segments.push_back(Segment("front",  Pose()));
  this->segments.push_back(Segment("middle", Pose()));
  this->segments.push_back(Segment("rear",   Pose()));

  /********************************************************/
  /*                   Front Segment                      */
  /********************************************************/
  this->segments[0].connectLeg(Leg("FL", 0, Vector3d(0.2, 0.3, 0.4), Pose(Vector3d( EIGEN_PI/2,0,0),Vector3d(-0.15,0,0))));
  this->segments[0].connectLeg(Leg("FR", 0, Vector3d(0.2, 0.3, 0.4), Pose(Vector3d(-EIGEN_PI/2,0,0),Vector3d( 0.15,0,0))));
  this->segments[0].connectJoint(Joint("j_1_2", 0, Joint::JOINT_TYPE::ROTATION_1D), true);


  /********************************************************/
  /*                   Middle Segment                     */
  /********************************************************/
  this->segments[1].connectLeg(Leg("ML", 0, Vector3d(0.2, 0.3, 0.4), Pose(Vector3d( EIGEN_PI/2,0,0),Vector3d(-0.15,0,0))));
  this->segments[1].connectLeg(Leg("MR", 0, Vector3d(0.2, 0.3, 0.4), Pose(Vector3d(-EIGEN_PI/2,0,0),Vector3d( 0.15,0,0))));
  this->segments[1].connectJoint(segments[0].joints[0], false);
  this->segments[1].connectJoint(Joint("j_2_3", 0, Joint::JOINT_TYPE::ROTATION_1D), true);


  /********************************************************/
  /*                     Rare Segment                     */
  /********************************************************/
  this->segments[2].connectLeg(Leg("RL", 0, Vector3d(0.2, 0.3, 0.4), Pose(Vector3d( EIGEN_PI/2,0,0),Vector3d(-0.15,0,0))));
  this->segments[2].connectLeg(Leg("RR", 0, Vector3d(0.2, 0.3, 0.4), Pose(Vector3d(-EIGEN_PI/2,0,0),Vector3d( 0.15,0,0))));
  this->segments[2].connectJoint(segments[1].joints[1], false);
}

Articulated::~Articulated()
{}