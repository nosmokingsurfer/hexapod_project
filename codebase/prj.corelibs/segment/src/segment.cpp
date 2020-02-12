#include <segment/segment.h>


Segment::Segment()
{
  this->name = "none";

}

Segment::Segment(const std::string& name_, const Pose& pose_)
{
  this->name = name_;
  this->segmentRF = pose_;
  this->initialized = true;

  this->mass = 0;//95.16;
  this->totalMass = 0;
  this->centerOfMass = Vector3d(0,0,0);
}


Segment::Segment(const Segment& L)
{
  this->fbIndex = L.fbIndex;
  this->initialized = L.initialized;
  this->joints = L.joints;
  this->legs = L.legs;

  for (auto i = 0; i < this->joints.size(); i++)
  {
    this->joints[i].setParentSegment(*this);
  }

  for(auto i = 0; i < this->legs.size(); i++)
  {
    this->legs[i].setParentSegment(*this);
  }

  
  this->name = L.name;
  this->segmentRF = L.segmentRF;

  this->mass = L.mass;
  this->totalMass = L.mass;
  this->centerOfMass = L.centerOfMass;
  this->momentsOfInetrcia = L.momentsOfInetrcia;
}

Segment::Segment(const std::string& name_, const Pose& pose_, const double mass_, const Vector3d& centerOfMass_)
{
  *this = Segment(name_, pose_);

  this->mass = mass_;
  this->totalMass = 0;
  this->centerOfMass = centerOfMass_;
}

Segment & Segment::operator=(const Segment& L)
{
  this->fbIndex = L.fbIndex;
  this->initialized = L.initialized;
  this->joints = L.joints;

  for (auto i = 0; i < this->joints.size(); i++)
  {
    this->joints[i].setParentSegment(*this);
  }

  for(auto i = 0; i < this->legs.size(); i++)
  {
    this->legs[i].setParentSegment(*this);
  }

  this->legs = L.legs;
  this->name = L.name;
  this->segmentRF = L.segmentRF;
  this->mass = L.mass;
  this->totalMass = L.totalMass;
  this->centerOfMass = L.centerOfMass;

  return *this;
}

Segment::~Segment()
{}

Vector3d Segment::getCOMcoords(void)
{
  Vector3d result(0,0,0);

  this->totalMass = 0;
  this->totalMass += this->mass;

  result = this->mass*this->centerOfMass;

  for (auto i = 0; i < this->legs.size(); i++)
  {
    this->totalMass += legs[i].getTotalMass();
    result += legs[i].getTotalMass()*(legs[i].pose.inverse().T*legs[i].getCOMcoords());
  }

  result /= totalMass;

  return result;
}

double Segment::getTotalMass()
{
  double result = 0;
  result = this->mass;

  for (auto i = 0; i < legs.size(); i++)
  {
    result += legs[i].getTotalMass();
  }

  this->totalMass = result;

  return result;
}

bool Segment::connectLeg(Leg leg)
{
  //leg.setParentSegment(*this);
  this->legs.push_back(leg);
  this->legs.back().setParentSegment(*this);
  return true;
}

bool Segment::connectJoint(Joint& joint, const bool parent)
{
  if (parent)
  {
    //joint.setParentSegment(*this);
    this->joints.push_back(joint);
    this->joints.back().setParentSegment(*this);
    return true;
  }
  else
  {
  //TODO implement connection to the child
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
