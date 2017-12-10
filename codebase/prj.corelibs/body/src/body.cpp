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
  Vector3d FLmOrient(0, 0, 0);
  Vector3d MLmOrient(0, 0, 0);
  Vector3d RLmOrient(0, 0, 0);
  Vector3d FRmOrient(0, 0, 0);
  Vector3d MRmOrient(0, 0, 0);
  Vector3d RRmOrient(0, 0, 0);

  for(int i = 0; i < 6; i++)
  {
    legs.push_back(Leg());
  }

  legs[0].init("FL", segments, FRmPoint, FRmOrient);
  legs[1].init("ML", segments, MRmPoint, MRmOrient);
  legs[2].init("RL", segments, RRmPoint, RRmOrient);
  legs[3].init("FR", segments, FLmPoint, FLmOrient);
  legs[4].init("MR", segments, MLmPoint, MLmOrient);
  legs[5].init("RR", segments, RLmPoint, RLmOrient);

}

Body::~Body()
{}

bool Body::getFB(VectorXd& feedback)
{
  this->FBcoords = feedback.segment<6>(this->fbIndex);
  this->FBvelocities = feedback.segment<6>(this->fbIndex + 6);

  return true;
}

VectorXd Body::getTargetPosition(double time)
{
  VectorXd result(6);

  result << 0,
    0,
    0,
    0,
    0,
    0;

  return result;
}
