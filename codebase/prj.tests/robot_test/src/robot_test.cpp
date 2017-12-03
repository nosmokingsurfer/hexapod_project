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

  Robot myRobot;

  VectorXd result;

  double inputArray[48];
  for(int i = 0; i < 48; i++)
  {
    inputArray[i] = i/100.0;
  }

  double parameters[3];
  for (int i = 0; i < 3; i++)
  {
    parameters[i] = i;
  }

  parameters[0] = 30;
  parameters[1] = 0;
  parameters[2] = 3;

  myRobot.recieveFeedBack(inputArray, 48);
  myRobot.recieveParameters(parameters, 3);


  result = myRobot.getControls(100);

  cout << result << endl;

	 return 0;
}

