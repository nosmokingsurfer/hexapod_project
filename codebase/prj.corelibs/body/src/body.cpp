#include <body/body.h>


Body::Body()
{
  this->fbIndex = 0;
  this->segments.clear();
}

Body::Body(BODY_TYPE bt)
{
  *this = Body();

  switch(bt)
  {
  case(BODY_TYPE::SIMPLE):
    {
      initSimpleBody();
      break;
    }
  case(BODY_TYPE::ARTICULATED):
    {
      initArticulatedBody();
      break;
    }
  case(BODY_TYPE::MOZAIK):
    {
      initMozaikBody();
      break;
    }
  }

  this->body_type = bt;

}

Body::~Body()
{}

void Body::initSimpleBody()
{
  Vector3d legSegments(0.05, 0.05, 0.05);

  this->segments.push_back(Segment("main", Pose()));

  this->segments[0].connectLeg(Leg("FL", 12, legSegments, Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-0.05,  0.1, 0))));
  this->segments[0].connectLeg(Leg("ML", 18, legSegments, Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-0.05,    0, 0))));
  this->segments[0].connectLeg(Leg("RL", 24, legSegments, Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-0.05, -0.1, 0))));
  this->segments[0].connectLeg(Leg("FR", 30, legSegments, Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( 0.05,  0.1, 0))));
  this->segments[0].connectLeg(Leg("MR", 36, legSegments, Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( 0.05,    0, 0))));
  this->segments[0].connectLeg(Leg("RR", 42, legSegments, Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( 0.05, -0.1, 0))));
}

void Body::initArticulatedBody()
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

void Body::initMozaikBody()
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

VectorXd Body::getSimpleBodyControls(double time)
{
  VectorXd targetAngles(18);
  targetAngles.fill(0);

  std::vector<Vector3d> goals;

  //положение следовых точек в абсолютной системе координат
  goals.push_back(Vector3d(-0.12,  0.1, 0.0)); //FL
  goals.push_back(Vector3d(-0.12,  0.0, 0.0)); //ML
  goals.push_back(Vector3d(-0.12, -0.1, 0.0)); //RL
  goals.push_back(Vector3d( 0.12,  0.1, 0.0)); //FR
  goals.push_back(Vector3d( 0.12,  0.0, 0.0)); //MR
  goals.push_back(Vector3d( 0.12, -0.1, 0.0)); //RR

  this->tarPose = getTargetPose(time);

  for (int i = 0; i < 6; i++)
  {
    //дл€ i-ноги считаем радиус вектор в —  св€занной с ногой
    Vector3d result = segments[0].legs[i].pose.T*tarPose.T*goals[i];
    targetAngles.segment(3*i, 3) = segments[0].legs[i].inverseKinematics(result);
  }
  return targetAngles;
}

VectorXd Body::getArticulatedBodyControls(double time)
{
  VectorXd result;

  cout << "NOT IMPLEMENTED: " << "Body::getArticulatedBodyControls" << endl;

  return result;
}

VectorXd Body::getMozaikBodyControls(double time)
{
  VectorXd result;

  cout << "NOT IMPLEMENTED: " << "Body::getMozaikBodyControls" << endl;

  return result;
}

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
    0.005*sin(2*EIGEN_PI/3*time),
    0.005*sin(2*EIGEN_PI/3*time),
    0.025 + 0.005*sin(2*EIGEN_PI/4*time);

   Vector3d YawPitchRoll;
    YawPitchRoll << 
      0.05*sin(2*EIGEN_PI/2*time),
      0.05*sin(2*EIGEN_PI/3*time),
      0.05*sin(2*EIGEN_PI/4*time);

  return Pose(YawPitchRoll, xyz);
}

VectorXd Body::getControls(double time)
{
  VectorXd result;
  switch(this->body_type)
  {
  case(BODY_TYPE::SIMPLE):
    {
      result = getSimpleBodyControls(time);
      break;
    }
  case(BODY_TYPE::ARTICULATED):
    {
      result = getArticulatedBodyControls(time);
      break;
    }
  case(BODY_TYPE::MOZAIK):
    {
      result = getMozaikBodyControls(time);
      break;
    }
  }
  return result;
}

void Body::printOut()
{
  for (int i = 0; i < this->segments.size(); i++)
  {
    cout << this->segments[i].getName() << endl;
    for (int j = 0; j < this->segments[i].legs.size(); j++)
    {
      cout << "\t" << this->segments[i].legs[j].getName() << endl;
      cout << "\tparent: " << this->segments[i].legs[j].getParentName() << endl;
    } 
    for(int j = 0; j < this->segments[i].joints.size(); j++)
    {
      cout << this->segments[i].joints[j].getName() << endl;
      cout << "\tparent: " << this->segments[i].joints[j].getParentName() << endl;
      cout << "\tchild: " << this->segments[i].joints[j].getChildName() << endl;
    }
  }
}
