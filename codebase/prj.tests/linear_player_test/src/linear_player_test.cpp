// linear_player_test.cpp : Defines the entry point for the console application.
//


#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <linear_player/linear_player.h>

using namespace std;
using namespace Eigen;


int main(int argc, char** argv)
{
  Vector3d A(0,0,0);

  Vector3d B(10,20,30);

  double T = 1;


  LinearPlayer player(Vector3d(0.5, 0.5, 0.1), Vector3d(0.5, 1.0, 0.1), 1.0, 1);
  
  double dt = 0.01;

  for (auto i = 90; i <= 210 ; i++)
  {
    player.getCurTargetState(i*dt);
  }

	 return 0;
}

