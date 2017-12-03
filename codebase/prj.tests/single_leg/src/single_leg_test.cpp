// single_leg_test.cpp : Defines the entry point for the console application.
//

#include <leg/leg.h>


#include <vector>

using namespace std;


int main(int argc, char** argv)
{

  Leg testLeg;

  vector<double> segs;
  segs.push_back(1);
  segs.push_back(2);
  segs.push_back(3);

  Eigen::Vector4d mountingPoint(0,0,0,1);
  Eigen::Vector3d mountingAngles(0, 0, 0);


  testLeg.init(segs, mountingPoint, mountingAngles);

  Vector3d solution;
  solution.fill(0);



  Eigen::Vector3d traj = testLeg.trajectoryGenerator(0);

  testLeg.inverseKinematics(traj, solution);

  cout << solution << endl;

	 return 0;
}

