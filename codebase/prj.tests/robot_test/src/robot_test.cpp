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

  result = myRobot.getControls(100);

  cout << result << endl;

	 return 0;
}

