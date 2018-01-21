// single_leg_test.cpp : Defines the entry point for the console application.
//

#include <leg/leg.h>


#include <vector>

using namespace std;


int main(int argc, char** argv)
{

  

  Vector3d segs;
  segs.fill(0.05);

  Eigen::Vector3d mountingPoint(0,0,0);
  Eigen::Vector3d mountingAngles(0, 0, 0);

  Vector3d solution;
  solution.fill(0);

  Leg testLeg("The Leg", 0, 0, segs, Pose(mountingAngles,mountingPoint));

  double dt = 0.01;
  for(int i = 0; i < 1000; i++)
  {

    Eigen::Vector3d traj = testLeg.forwardKinematics(Vector3d(0.04*sin(i*dt*(EIGEN_PI*2/3)), 0.1 + 0.1*sin(i*dt*(EIGEN_PI*2/5)), -0.1 + 0.2*sin(i*dt*(EIGEN_PI*2/5))));

    traj = testLeg.trajectoryGenerator(i*dt);

    cout << "targetPoint = " << traj.transpose() << endl;

    solution = testLeg.inverseKinematics(traj);

    cout << "solution = " << solution.transpose() << endl;

    cout << "error = " << (traj - testLeg.forwardKinematics(solution)).transpose() << endl;

  }

	 return 0;
}

