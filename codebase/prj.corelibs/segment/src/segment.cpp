#include <segment/segment.h>


Segment::Segment()
{}

Segment::Segment(const std::string name_, const Pose& pose_)
{
  this->name = name_;
  this->segmentRF = pose_;
}

Segment::~Segment()
{}

bool Segment::init(const vector<Leg> &legs)
{
  this->legs = legs;
  return true;
}
