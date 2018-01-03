#include <robot/robot.h>

using namespace Eigen;

Robot::Robot()
{
  this->controlTorques = VectorXd(18);
  this->controlTorques.fill(0);

  this->feedBack = VectorXd(48);
  this->feedBack.fill(0);

  this->calculatedjoints = VectorXd(48);
  this->calculatedjoints.fill(0);
}

Robot::~Robot()
{

}

Eigen::VectorXd Robot::getControls(double time)
{
#if 0
  VectorXd targetAngles(18);
  targetAngles.fill(0);

  //каким-либо образом считаем целевые шарнирные углы
  for (int i = 0; i < 6; i++)
  {
    targetAngles.segment(3*i,3) = robotBody.legs[i].inverseKinematics(robotBody.legs[i].trajectoryGenerator(time));
  }

  this->calculatedjoints = targetAngles;

#else
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

  robotBody.tarPose = robotBody.getTargetPose(time);

  for (int i = 0; i < 6; i++)
  {
    //для i-ноги считаем радиус вектор в СК связанной с ногой
    Vector3d result = robotBody.segments[0].legs[i].pose.T*robotBody.tarPose.T*goals[i];
    targetAngles.segment(3*i, 3) = robotBody.segments[0].legs[i].inverseKinematics(result);
  }
    this->calculatedjoints = targetAngles;


#endif




  //по рассчитанным углам считаем моменты
  VectorXd torques(controlTorques.size());
  torques.fill(0);

  for (int i = 0; i < 6; i++)
  {
    torques.segment(3*i, 3) = robotBody.segments[0].legs[i].getTorques(targetAngles.segment(3*i, 3));
  }
  
  this->controlTorques = torques;
  return torques;
}

Eigen::VectorXd Robot::getCalculatedJoints()
{
  return this->calculatedjoints;
}

bool Robot::recieveFeedBack(double* inputs, int numberOfInputs)
{
  Eigen::VectorXd signals(numberOfInputs);
  signals.fill(0);

  for (int i = 0; i < numberOfInputs; i++)
  {
    signals[i] = inputs[i];
  }

  this->feedBack = signals;


  this->robotBody.recieveFB(signals);

  for (int i = 0; i < static_cast<int>(robotBody.segments[0].legs.size()); i++)
  {
    robotBody.segments[0].legs[i].recieveFB(feedBack, 12 + i*6);
  }
  return true;
}

bool Robot::recieveParameters(double* params, int numberOfParams)
{
  Eigen::VectorXd parameters(numberOfParams);
  parameters.fill(0);

  for (int i = 0; i < numberOfParams; i++)
  {
    parameters[i] = params[i];
  }

  this->parameters = parameters;

  
  for(int j = 0; j < static_cast<int>(robotBody.segments.size()); j++)
  {
    for(int i = 0; i < static_cast<int>(robotBody.segments[j].legs.size()); i++)
    {
      PID::PIDcoeffs coeffs;
      coeffs.kP = parameters[0];
      coeffs.kI = parameters[1];
      coeffs.kD = parameters[2];
      robotBody.segments[j].legs[i].setCoeffs(coeffs);
    }
  }

  return true;
}

