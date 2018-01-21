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


Segment::~Segment()
{}

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

bool Segment::recieveFB(const VectorXd fb)
{
  for(int i = 0; i < static_cast<int>(legs.size()); i++)
  {
    legs[i].recieveFB(fb);
  }

  for(int i = 0; i < static_cast<int>(joints.size()); i++)
  {
    joints[i].recieveFB(fb);
  }
  return true;
}

std::string Segment::getName()
{
  return this->name;
}
