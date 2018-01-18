#include <segment/segment.h>


Segment::Segment()
{
  this->name = "none";

}

Segment::Segment(const std::string name_, const Pose& pose_)
{
  this->name = name_;
  this->segmentRF = pose_;
  this->initialized = true;
}

Segment::Segment(const Segment& L)
{
  this->name = L.name;
  this->initialized = L.initialized;
  this->segmentRF = L.segmentRF;
  this->fbIndex = L.fbIndex;
  this->legs = L.legs;
  this->joints = L.joints;
}

Segment & Segment::operator=(const Segment& L)
{
  if (this == &L) return (*this);
  else
  {
    this->name = L.name;
    this->initialized = L.initialized;
    this->segmentRF = L.segmentRF;
    this->fbIndex = L.fbIndex;
    this->legs = L.legs;
    this->joints = L.joints;

    return *this;
  }
}


Segment::~Segment()
{}

bool Segment::init(const vector<Leg> &legs, const vector<Joint> &joints)
{
  this->legs = legs;
  this->joints = joints;

  return true;
}

bool Segment::connectLeg(Leg& leg)
{
  leg.setParentSegment(*this);
  this->legs.push_back(leg);
  return true;
}

bool Segment::connectJoint(Joint& joint, const bool parent)
{
  if (parent)
  {
    joint.setParentSegment(*this);
    this->joints.push_back(joint);
    return true;
  }
  else
  {
    joint.setChildSegment(*this);
    //this->joints.push_back(joint); нужно ли это делать?
    return true;
  }
}

bool Segment::isInitialized()
{
  return this->initialized;
}

std::string Segment::getName()
{
  return this->name;
}
