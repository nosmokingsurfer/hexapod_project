#include <simple_body/simple_body.h>


SimpleBody::SimpleBody()
{
  this->fbIndex = 0;
  Vector3d legSegments(0.05, 0.05, 0.05);

  this->segments.push_back(Segment("main", Pose()));

  this->segments[0].connectLeg(Leg("FL", 0, legSegments, Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-0.05,  0.1, 0))));
  this->segments[0].connectLeg(Leg("ML", 0, legSegments, Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-0.05,    0, 0))));
  this->segments[0].connectLeg(Leg("RL", 0, legSegments, Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-0.05, -0.1, 0))));
  this->segments[0].connectLeg(Leg("FR", 0, legSegments, Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( 0.05,  0.1, 0))));
  this->segments[0].connectLeg(Leg("MR", 0, legSegments, Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( 0.05,    0, 0))));
  this->segments[0].connectLeg(Leg("RR", 0, legSegments, Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( 0.05, -0.1, 0))));
}

SimpleBody::~SimpleBody()
{}
