// single_leg_test.cpp : Defines the entry point for the console application.
//

#include <robot/robot.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

using namespace std;
using namespace Eigen;


int main(int argc, char** argv)
{

  Robot myRobot(Body::BODY_TYPE::ARTICULATED);

  VectorXd result;
  
  double inputArray[52];
  for(int i = 0; i < 52; i++)
  {
    inputArray[i] = 0.05*i;
  }

  inputArray[0] = 0.0;
  inputArray[1] = 0.0;
  inputArray[2] = 0.05;
  inputArray[3] = 0.0;
  inputArray[4] = 0.0;
  inputArray[5] = 0.0;


  double parameters[3];
  for (int i = 0; i < 3; i++)
  {
    parameters[i] = i;
  }

  parameters[0] = 30;
  parameters[1] = 0;
  parameters[2] = 3;

  myRobot.recieveFeedBack(inputArray, 52);
  myRobot.recieveParameters(parameters, 3);  


  double dt = 0.1;

  for (int i = 0; i < 30; i++)
  {
    result = myRobot.getControls(i*dt);
  }

  cout << "Center of mass = " << myRobot.getCOM() << endl;
  cout << "Total mass = " << myRobot.getTotalMass() << endl;


  Vector3d segments(1,2,3);
  std::vector<double> masses;
  masses.push_back(60.26);
  masses.push_back(120.5);
  masses.push_back(180.8);


  Leg testLeg("testLeg",0,0,0,segments,Pose(),masses);

  VectorXd fb(6);
  fb << 0.78539816339744830961566084581988,0.78539816339744830961566084581988,0.78539816339744830961566084581988,0,0,0;

  testLeg.recieveFB(fb);

  result = testLeg.getCOMcoords();

  cout << result << endl;



	 return 0;
}

