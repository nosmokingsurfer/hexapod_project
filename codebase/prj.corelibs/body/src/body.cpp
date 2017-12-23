#include <body/body.h>


Body::Body()
{
  this->fbIndex = 0;


  //legs dimensions
  Vector3d segments(0.05, 0.05, 0.05);

  //TODO zip mounting point and mounting orientation in one class - will be able to iterate through it
  //legs mounting points
  Vector3d FLmPoint(-0.05,  0.1, 0);
  Vector3d MLmPoint(-0.05,    0, 0);
  Vector3d RLmPoint(-0.05, -0.1, 0);
  Vector3d FRmPoint( 0.05,  0.1, 0);
  Vector3d MRmPoint( 0.05,    0, 0);
  Vector3d RRmPoint( 0.05, -0.1, 0);


  //legs mounting orientations
  Vector3d FLmOrient( EIGEN_PI/2, 0, 0);
  Vector3d MLmOrient( EIGEN_PI/2, 0, 0);
  Vector3d RLmOrient( EIGEN_PI/2, 0, 0);
  Vector3d FRmOrient(-EIGEN_PI/2, 0, 0);
  Vector3d MRmOrient(-EIGEN_PI/2, 0, 0);
  Vector3d RRmOrient(-EIGEN_PI/2, 0, 0);

  for(int i = 0; i < 6; i++)
  {
    legs.push_back(Leg());
  }

  legs[0].init("FL", segments, FLmPoint, FLmOrient);
  legs[1].init("ML", segments, MLmPoint, MLmOrient);
  legs[2].init("RL", segments, RLmPoint, RLmOrient);
  legs[3].init("FR", segments, FRmPoint, FRmOrient);
  legs[4].init("MR", segments, MRmPoint, MRmOrient);
  legs[5].init("RR", segments, RRmPoint, RRmOrient);

}

Body::~Body()
{}

bool Body::recieveFB(VectorXd& feedback)
{
  this->FBcoords = feedback.segment<6>(this->fbIndex);
  this->FBvelocities = feedback.segment<6>(this->fbIndex + 6);

  this->fbPose = Pose(FBcoords.tail(3), FBcoords.head(3));

  return true;
}

Pose Body::getTargetPose(double time)
{
  Vector3d xyz;

   xyz <<
    0,
    0,
    0.05 + 0.005*sin(2*EIGEN_PI/4*time);

   Vector3d YawPitchRoll;
    YawPitchRoll << 
      0.05*sin(2*EIGEN_PI/2*time),
      0.05*sin(2*EIGEN_PI/3*time),
      0.05*sin(2*EIGEN_PI/4*time);

  return Pose(YawPitchRoll, xyz);
}
