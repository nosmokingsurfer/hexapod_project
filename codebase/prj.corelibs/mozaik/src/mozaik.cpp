#include <mozaik/mozaik.h>


Mozaik::Mozaik()
{
  ///the body initialization order as follows:
  /// 1) all segments - w/o legs and joints
  /// 2) all legs - connect to segments: define parents and childrens in pointers
  /// 3) all joints between segments - will be able to define all parents and childrens

  //////////////////////////////////////////////////////////////////////////
  // Defining all segments of the body
  //
  //
  //       FL - s_3 --> j_3_4 >-- s_4 - FR
  //             |                 |
  //             ^                \|/
  //           j_2_3             j_4_5
  //             ^                \|/
  //             |                 |
  //       ML - s_2 --> j_2_5 >-- s_5 - MR
  //             |                 |
  //             ^                \|/
  //           j_1_2             j_5_6
  //             ^                \|/
  //             |                 |
  //       RL - s_1 --< j_6_1 <-- s_6 - RR
  //
  //
  //////////////////////////////////////////////////////////////////////////

  //Segments
  this->segments.clear();
  this->segments.push_back(Segment("s_1", Pose()));
  this->segments.push_back(Segment("s_2", Pose()));
  this->segments.push_back(Segment("s_3", Pose()));
  this->segments.push_back(Segment("s_4", Pose()));
  this->segments.push_back(Segment("s_5", Pose()));
  this->segments.push_back(Segment("s_6", Pose()));

  //Legs
  double a = 0.2;
  double sqrt_2 = sqrt(2);
  this->segments[2].connectLeg(Leg("FL", 0, Vector3d(0.05,0.1,0.2), Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-sqrt_2*a,0,0))));
  this->segments[1].connectLeg(Leg("ML", 0, Vector3d(0.05,0.1,0.2), Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-sqrt_2*a,0,0))));
  this->segments[0].connectLeg(Leg("RL", 0, Vector3d(0.05,0.1,0.2), Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-sqrt_2*a,0,0))));
  this->segments[3].connectLeg(Leg("FR", 0, Vector3d(0.05,0.1,0.2), Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( sqrt_2*a,0,0))));
  this->segments[4].connectLeg(Leg("MR", 0, Vector3d(0.05,0.1,0.2), Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( sqrt_2*a,0,0))));
  this->segments[5].connectLeg(Leg("RR", 0, Vector3d(0.05,0.1,0.2), Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( sqrt_2*a,0,0))));

  //Joints
  this->segments[0].connectJoint(Joint("j_1_2", 0, Joint::JOINT_TYPE::ROTATION_3D), true);
  this->segments[1].connectJoint(this->segments[0].joints[0], false);

  this->segments[1].connectJoint(Joint("j_2_3", 0, Joint::JOINT_TYPE::ROTATION_3D), true);
  this->segments[2].connectJoint(this->segments[1].joints[0], false);

  this->segments[2].connectJoint(Joint("j_3_4", 0, Joint::JOINT_TYPE::ROTATION_3D), true);
  this->segments[3].connectJoint(this->segments[2].joints[0], false);

  this->segments[3].connectJoint(Joint("j_4_5", 0, Joint::JOINT_TYPE::ROTATION_3D), true);
  this->segments[4].connectJoint(this->segments[3].joints[0], false);

  this->segments[4].connectJoint(Joint("j_5_6", 0, Joint::JOINT_TYPE::ROTATION_3D), true);
  this->segments[5].connectJoint(this->segments[4].joints[0], false);

  this->segments[5].connectJoint(Joint("j_6_1", 0, Joint::JOINT_TYPE::ROTATION_3D), true);
  this->segments[0].connectJoint(this->segments[5].joints[0], false);

  this->segments[1].connectJoint(Joint("j_2_5", 0, Joint::JOINT_TYPE::ROTATION_3D), true);
  this->segments[4].connectJoint(this->segments[1].joints[1], false);
  
}

Mozaik::~Mozaik()
{}



bool Mozaik::getAxis(const Joint& j1, const Joint& j2, Vector3d& axis1, Vector3d& axis2)
{
  axis1.fill(0);
  axis2.fill(0);
  //TODO
  return true;
}

bool Mozaik::rotateJoint(Joint& j1, const Vector3d& axis, const double angle)
{
  return true;
}

bool Mozaik::rotateAroundAxis(Joint& j1, Joint& j2, double angle)
{
  return true;
}

bool Mozaik::updateBodyGeometry(const BodyCoordinates& bodyConfig)
{
  return true;
}
