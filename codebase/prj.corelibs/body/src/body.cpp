#include <body/body.h>

Body::Body()
{

}

Body::Body(BODY_TYPE bt)
{
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

  this->segments[0].connectLeg(Leg("FL", 12,  0, 18, legSegments, Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-0.05,  0.1, 0))));
  this->segments[0].connectLeg(Leg("ML", 18,  3, 21, legSegments, Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-0.05,    0, 0))));
  this->segments[0].connectLeg(Leg("RL", 24,  6, 24, legSegments, Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-0.05, -0.1, 0))));
  this->segments[0].connectLeg(Leg("FR", 30,  9, 27, legSegments, Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( 0.05,  0.1, 0))));
  this->segments[0].connectLeg(Leg("MR", 36, 12, 30, legSegments, Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( 0.05,    0, 0))));
  this->segments[0].connectLeg(Leg("RR", 42, 15, 33, legSegments, Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( 0.05, -0.1, 0))));

  this->nDOF = 24;
  this->nControls = 36;
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
  //       FL - front - FR
  //              |   
  //              p   
  //            j_1_2 
  //              c   
  //              |   
  //       ML -  mid  - MR
  //              |   
  //              p   
  //            j_2_3 
  //              c   
  //              |   
  //       RL - rear  - RR  
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
  this->segments[0].connectLeg(Leg("FL", 12, 0, 18, Vector3d(0.2, 0.3, 0.4), Pose(Vector3d( EIGEN_PI/2,0,0),Vector3d(-0.15,0,0))));
  this->segments[0].connectLeg(Leg("FR", 30, 9, 27, Vector3d(0.2, 0.3, 0.4), Pose(Vector3d(-EIGEN_PI/2,0,0),Vector3d( 0.15,0,0))));
  this->segments[0].connectJoint(Joint("j_1_2", 48, 36, 38, Joint::JOINT_TYPE::ROTATION_1D, Pose(Vector3d(0,0,0), Vector3d(0,-0.25,0)), Pose(Vector3d(0,0,0),Vector3d(0,0.25,0))), true);


  /********************************************************/
  /*                   Middle Segment                     */
  /********************************************************/
  this->segments[1].connectLeg(Leg("ML", 18, 3, 21, Vector3d(0.2, 0.3, 0.4), Pose(Vector3d( EIGEN_PI/2,0,0),Vector3d(-0.15,0,0))));
  this->segments[1].connectLeg(Leg("MR", 36, 12, 30, Vector3d(0.2, 0.3, 0.4), Pose(Vector3d(-EIGEN_PI/2,0,0),Vector3d( 0.15,0,0))));
  this->segments[1].connectJoint(segments[0].joints[0], false);
  this->segments[1].connectJoint(Joint("j_2_3", 50, 37, 39, Joint::JOINT_TYPE::ROTATION_1D, Pose(Vector3d(0,0,0), Vector3d(0,-0.25,0)), Pose(Vector3d(0,0,0),Vector3d(0,0.25,0))), true);


  /********************************************************/
  /*                     Rear Segment                     */
  /********************************************************/
  this->segments[2].connectLeg(Leg("RL", 24, 6, 24, Vector3d(0.2, 0.3, 0.4), Pose(Vector3d( EIGEN_PI/2,0,0),Vector3d(-0.15,0,0))));
  this->segments[2].connectLeg(Leg("RR", 42, 15, 33, Vector3d(0.2, 0.3, 0.4), Pose(Vector3d(-EIGEN_PI/2,0,0),Vector3d( 0.15,0,0))));
  this->segments[2].connectJoint(segments[1].joints[0], false);

  this->nDOF = 26;
  this->nControls = 40;


  std::string fileName = "D:/_my_phd/codebase/prj.control_dlls/articulated_dll/articulated_controls.json";
  this->controlCommands.init(fileName);

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
  this->segments[2].connectLeg(Leg("FL", 0, 0, 0, Vector3d(0.05,0.1,0.2), Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-sqrt_2*a,0,0))));
  this->segments[1].connectLeg(Leg("ML", 0, 0, 0, Vector3d(0.05,0.1,0.2), Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-sqrt_2*a,0,0))));
  this->segments[0].connectLeg(Leg("RL", 0, 0, 0, Vector3d(0.05,0.1,0.2), Pose(Vector3d( EIGEN_PI/2, 0, 0), Vector3d(-sqrt_2*a,0,0))));
  this->segments[3].connectLeg(Leg("FR", 0, 0, 0, Vector3d(0.05,0.1,0.2), Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( sqrt_2*a,0,0))));
  this->segments[4].connectLeg(Leg("MR", 0, 0, 0, Vector3d(0.05,0.1,0.2), Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( sqrt_2*a,0,0))));
  this->segments[5].connectLeg(Leg("RR", 0, 0, 0, Vector3d(0.05,0.1,0.2), Pose(Vector3d(-EIGEN_PI/2, 0, 0), Vector3d( sqrt_2*a,0,0))));

  //Joints
  this->segments[0].connectJoint(Joint("j_1_2", 0,0,0, Joint::JOINT_TYPE::ROTATION_3D, Pose(Vector3d(0,0,0),Vector3d(0,sqrt_2*a,0)), Pose(Vector3d(0,0,0),Vector3d(0,-sqrt_2*a,0))), true);
  this->segments[1].connectJoint(this->segments[0].joints[0], false);

  this->segments[1].connectJoint(Joint("j_2_3", 0,0,0, Joint::JOINT_TYPE::ROTATION_3D, Pose(Vector3d(0,0,0),Vector3d(0,sqrt_2*a,0)), Pose(Vector3d(0,0,0),Vector3d(0,-sqrt_2*a,0))), true);
  this->segments[2].connectJoint(this->segments[1].joints[0], false);

  this->segments[2].connectJoint(Joint("j_3_4", 0,0,0, Joint::JOINT_TYPE::ROTATION_3D, Pose(Vector3d(-EIGEN_PI/2,0,0), Vector3d(sqrt_2*a,0,0)), Pose(Vector3d(EIGEN_PI/2,0,0),Vector3d(-sqrt_2*a,0,0))), true);
  this->segments[3].connectJoint(this->segments[2].joints[0], false);

  this->segments[3].connectJoint(Joint("j_4_5", 0,0,0, Joint::JOINT_TYPE::ROTATION_3D, Pose(Vector3d(EIGEN_PI,0,0),Vector3d(0,-sqrt_2*a,0)),Pose(Vector3d(0,0,0), Vector3d(0,sqrt_2*a,0))), true);
  this->segments[4].connectJoint(this->segments[3].joints[0], false);

  this->segments[4].connectJoint(Joint("j_5_6", 0,0,0, Joint::JOINT_TYPE::ROTATION_3D, Pose(Vector3d(EIGEN_PI,0,0),Vector3d(0,-sqrt_2*a,0)),Pose(Vector3d(0,0,0), Vector3d(0,sqrt_2*a,0))), true);
  this->segments[5].connectJoint(this->segments[4].joints[0], false);

  this->segments[5].connectJoint(Joint("j_6_1", 0,0,0, Joint::JOINT_TYPE::ROTATION_3D, Pose(Vector3d(EIGEN_PI/2,0,0),Vector3d(-sqrt_2*a,0,0)), Pose(Vector3d(-EIGEN_PI/2,0,0),Vector3d(sqrt_2*a,0,0))), true);
  this->segments[0].connectJoint(this->segments[5].joints[0], false);

  this->segments[1].connectJoint(Joint("j_2_5", 0,0,0, Joint::JOINT_TYPE::ROTATION_3D, Pose(Vector3d(-EIGEN_PI/2,0,0), Vector3d(sqrt_2*a,0,0)), Pose(Vector3d(EIGEN_PI/2,0,0),Vector3d(-sqrt_2*a,0,0))), true);
  this->segments[4].connectJoint(this->segments[1].joints[1], false);

  this->nDOF = 28; //TODO check this number
  this->nControls = 39; //TODO check this number

  std::string fileName = "D:/_my_phd/codebase/prj.control_dlls/articulated_dll/articulated_controls.json";
  this->controlCommands.init(fileName);
}

VectorXd Body::getSimpleBodyControlAngles(double time)
{
  VectorXd targetAngles(nControls);
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
    //для i-ноги считаем радиус вектор в СК связанной с ногой
    Vector3d result = segments[0].legs[i].pose.T*tarPose.T*goals[i];
    targetAngles.segment(segments[0].legs[i].getDebIndex(), 3) = segments[0].legs[i].inverseKinematics(result);
    segments[0].legs[i].setTargetState(targetAngles.segment(segments[0].legs[i].getDebIndex(), 3));
  }
  return targetAngles;
}

VectorXd Body::getArticulatedBodyControlAngles(double time)
{
  VectorXd result(nControls);
  result.fill(0);

  //положение следовых точек в абсолютной системе координат
  //TODO get points from planner
  Vector3d FL_goal(-0.5,  0.5, 0.0); //FL
  Vector3d ML_goal(-0.5,  0.3, 0.0); //ML
  Vector3d RL_goal(-0.5, -0.5, 0.0); //RL
  Vector3d FR_goal( 0.5,  0.5, 0.0); //FR
  Vector3d MR_goal( 0.5,  0.3, 0.0); //MR
  Vector3d RR_goal( 0.5, -0.5, 0.0); //RR
  
    
  this->tarPose = getTargetPose(0);

  VectorXd j_1_2_state(1);
  j_1_2_state << 0;

  VectorXd j_2_3_state(1);
  j_2_3_state << 0;
 
  FL_goal = this->controlCommands.getControls("FL", time);
  FR_goal = this->controlCommands.getControls("FR", time);
  ML_goal = this->controlCommands.getControls("ML", time);
  MR_goal = this->controlCommands.getControls("MR", time);
  RL_goal = this->controlCommands.getControls("RL", time);
  RR_goal = this->controlCommands.getControls("RR", time);

  j_1_2_state = this->controlCommands.getControls("j_1_2", time);
  j_2_3_state = this->controlCommands.getControls("j_2_3", time);

  tarPose = Pose(this->controlCommands.getControls("seg_angles", time), this->controlCommands.getControls("seg_xyz",time));

  this->segments[0].joints[0].setTargetState(j_1_2_state);
  this->segments[1].joints[0].setTargetState(j_2_3_state);

  result.segment(segments[0].joints[0].getDebIndex(), 1) = segments[0].joints[0].getTargetState();
  result.segment(segments[1].joints[0].getDebIndex(), 1) = segments[1].joints[0].getTargetState();


  Vector3d temp;
  //FL
  temp =  segments[0].legs[0].pose.T*
          segments[0].joints[0].getMountInParent().T.inverse()*
          segments[0].joints[0].getTransformation().T* //TODO check the transformation
          segments[0].joints[0].getMountInChild().T*
          tarPose.T*
          FL_goal;

  segments[0].legs[0].setTargetState(segments[0].legs[0].inverseKinematics(temp));
  segments[0].legs[0].setTargetState(segments[0].legs[0].numericalSolve(temp));
  result.segment(segments[0].legs[0].getDebIndex(), 3) = segments[0].legs[0].inverseKinematics(temp);

  //FR
  temp =  segments[0].legs[1].pose.T*
          segments[0].joints[0].getMountInParent().T.inverse()*
          segments[0].joints[0].getTransformation().T* //TODO check the transformation
          segments[0].joints[0].getMountInChild().T*
          tarPose.T*
          FR_goal;
  
  segments[0].legs[1].setTargetState(segments[0].legs[1].inverseKinematics(temp));
  result.segment(segments[0].legs[1].getDebIndex(), 3) = segments[0].legs[1].inverseKinematics(temp);


  //ML
  temp =  segments[1].legs[0].pose.T*
          tarPose.T*
          ML_goal;

  segments[1].legs[0].setTargetState(segments[1].legs[0].inverseKinematics(temp));
  result.segment(segments[1].legs[0].getDebIndex(), 3) = segments[1].legs[0].inverseKinematics(temp);


  //MR
  temp =  segments[1].legs[1].pose.T*
          tarPose.T*
          MR_goal;

  segments[1].legs[1].setTargetState(segments[1].legs[1].inverseKinematics(temp));
  result.segment(segments[1].legs[1].getDebIndex(), 3) = segments[1].legs[1].inverseKinematics(temp);

  //RL
  temp =  segments[2].legs[0].pose.T*
          segments[1].joints[0].getMountInChild().T.inverse()*
          segments[1].joints[0].getTransformation().T* //TODO check the transformation
          segments[1].joints[0].getMountInParent().T*
          tarPose.T*
          RL_goal;

  segments[2].legs[0].setTargetState(segments[2].legs[0].inverseKinematics(temp));
  result.segment(segments[2].legs[0].getDebIndex(), 3) = segments[2].legs[0].inverseKinematics(temp);

  //RR
  temp =  segments[2].legs[1].pose.T*
          segments[1].joints[0].getMountInChild().T.inverse()*
          segments[1].joints[0].getTransformation().T* //TODO check the transformation
          segments[1].joints[0].getMountInParent().T*
          tarPose.T*
          RR_goal;

  segments[2].legs[1].setTargetState(segments[2].legs[1].inverseKinematics(temp));
  result.segment(segments[2].legs[1].getDebIndex(), 3) = segments[2].legs[1].inverseKinematics(temp);

  return result;
}

VectorXd Body::getMozaikBodyControlAngles(double time)
{
  VectorXd result(nControls);
  result.fill(0);


  //положение следовых точек в абсолютной системе координат
  //TODO get points from planner
  Vector3d FL_goal(-0.5,  0.5, 0.0); //FL
  Vector3d ML_goal(-0.5,  0.3, 0.0); //ML
  Vector3d RL_goal(-0.5, -0.5, 0.0); //RL
  Vector3d FR_goal( 0.5,  0.5, 0.0); //FR
  Vector3d MR_goal( 0.5,  0.3, 0.0); //MR
  Vector3d RR_goal( 0.5, -0.5, 0.0); //RR


  this->tarPose = getTargetPose(0);

  VectorXd j_1_2_state(3);
  j_1_2_state << 0,0,0;

  VectorXd j_2_3_state(3);
  j_2_3_state << 0,0,0;

  VectorXd j_3_4_state(3);
  j_3_4_state << 0,0,0;

  VectorXd j_4_5_state(3);
  j_4_5_state << 0,0,0;

  VectorXd j_5_6_state(3);
  j_5_6_state << 0,0,0;

  VectorXd j_6_1_state(3);
  j_6_1_state << 0,0,0;

  VectorXd j_2_5_state(3);
  j_2_5_state << 0,0,0;

  FL_goal = this->controlCommands.getControls("FL", time);
  FR_goal = this->controlCommands.getControls("FR", time);
  ML_goal = this->controlCommands.getControls("ML", time);
  MR_goal = this->controlCommands.getControls("MR", time);
  RL_goal = this->controlCommands.getControls("RL", time);
  RR_goal = this->controlCommands.getControls("RR", time);

  //j_1_2_state = this->controlCommands.getControls("j_1_2", time);
  //j_2_3_state = this->controlCommands.getControls("j_2_3", time);

  tarPose = Pose(this->controlCommands.getControls("seg_angles", time), this->controlCommands.getControls("seg_xyz",time));

  this->segments[0].joints[0].setTargetState(j_1_2_state);
  this->segments[1].joints[0].setTargetState(j_2_3_state);


  /*Mozaik mozaikControl;

  mozaikControl.updateBodyGeometry(body_angles);

  j_1_2_state = mozaikControl.getControlAngles("j_1_2", time);
  j_2_3_state = ...

  j_6_1_state = ...
  j_2_5_state = mozaikControl.getControlAngles("j_2_5", time);
  */




  result.segment(segments[0].joints[0].getDebIndex(), 3) = segments[0].joints[0].getTargetState();
  result.segment(segments[1].joints[0].getDebIndex(), 3) = segments[1].joints[0].getTargetState();
  
  return result;
}

Vector3d Body::getCOMcoords()
{
  Vector3d result(0,0,0);

  switch(this->body_type)
  {
    case(BODY_TYPE::SIMPLE):
    {
      result = getSimpleCOMcoords();
    }
    case(BODY_TYPE::ARTICULATED):
    {
      result = getArticulatedCOMcoords();
    }
    case(BODY_TYPE::MOZAIK):
    {
      result = getMozaikCOMcoords();
    }
  }

  return result;
}

Vector3d Body::getSimpleCOMcoords()
{
  Vector3d result(0,0,0);

  result = this->segments[0].getCOMcoords();

  this->totalMass = segments[0].totalMass;

  return result;
}

Vector3d Body::getArticulatedCOMcoords()
{
  Vector3d result(0,0,0);
  this->totalMass = 0;

  //center of mass of front segment in reference middle segment reference frame
  Vector3d front;
  front = segments[0].joints[0].getMountInChild().T.inverse()*
          segments[0].joints[0].getTransformation().T.inverse()*
          segments[0].joints[0].getMountInParent().T*
          segments[0].getCOMcoords();

  double frontMass = segments[0].totalMass;

  Vector3d middle;
  middle = segments[1].getCOMcoords();
  double middleMass = segments[1].totalMass;


  Vector3d rear;
  rear = segments[1].joints[1].getMountInParent().T.inverse()*
         segments[1].joints[0].getTransformation().T.inverse()*
         segments[1].joints[0].getMountInChild().T*
         segments[2].getCOMcoords();

  double rearMass = segments[2].totalMass;


  result = (frontMass*front + middleMass*middle + rearMass*rear)/(frontMass + middleMass + rearMass);

  return result;
}

Vector3d Body::getMozaikCOMcoords()
{
  Vector3d result(0,0,0);
  cout << " Body::getMozaikCOMcoords() - NOT IMPLEMENTED" << endl;
  return result;
}

bool Body::recieveFB(const VectorXd& feedback)
{
  for (int i = 0; i < static_cast<int>(segments.size()); i++)
  {
    segments[i].recieveFB(feedback);
  }
  return true;
}

Pose Body::getTargetPose(double time)
{
  Vector3d xyz;
  xyz.fill(0);

  Vector3d YawPitchRoll;
  YawPitchRoll.fill(0);

  switch(this->body_type)
  {
  case(BODY_TYPE::SIMPLE):
    {
     xyz <<
      0.005*sin(2*EIGEN_PI/3*time),
      0.005*sin(2*EIGEN_PI/3*time),
      0.025 + 0.005*sin(2*EIGEN_PI/4*time);

      YawPitchRoll << 
        0.05*sin(2*EIGEN_PI/2*time),
        0.05*sin(2*EIGEN_PI/3*time),
        0.05*sin(2*EIGEN_PI/4*time);    
      break;
    }
    case(BODY_TYPE::ARTICULATED):
    {
      xyz <<
        0.05*sin(2*EIGEN_PI/2*time),
        0.05*sin(2*EIGEN_PI/3*time),
        0.3 + 0.01*sin(2*EIGEN_PI/5*time);


      //xyz << 0, 0, 0.3;

      YawPitchRoll << 
        0.05*sin(2*EIGEN_PI/7*time),
        0.05*sin(2*EIGEN_PI/11*time),
        0.05*sin(2*EIGEN_PI/13*time);    

      //YawPitchRoll << 0,0,0;

      break;
    }
    case(BODY_TYPE::MOZAIK):
    {
      xyz <<
        0.05*sin(2*EIGEN_PI/2*time),
        0.05*sin(2*EIGEN_PI/3*time),
        0.3 + 0.01*sin(2*EIGEN_PI/5*time);


      //xyz << 0, 0, 0.3;

      YawPitchRoll << 
        0.05*sin(2*EIGEN_PI/7*time),
        0.05*sin(2*EIGEN_PI/11*time),
        0.05*sin(2*EIGEN_PI/13*time);    

      //YawPitchRoll << 0,0,0;

      break;
    }
  }
  return Pose(YawPitchRoll, xyz);
}

VectorXd Body::getControlAngles(double time)
{
  VectorXd result;
  switch(this->body_type)
  {
  case(BODY_TYPE::SIMPLE):
    {
      result = getSimpleBodyControlAngles(time);
      break;
    }
  case(BODY_TYPE::ARTICULATED):
    {
      result = getArticulatedBodyControlAngles(time);
      break;
    }
  case(BODY_TYPE::MOZAIK):
    {
      result = getMozaikBodyControlAngles(time);
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
    }
  }
}

int Body::getNDOF()
{
  return this->nDOF;
}

int Body::getNControls()
{
  return this->nControls;
}
