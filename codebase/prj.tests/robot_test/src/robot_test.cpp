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

	 return 0;
}

