// joint_test.cpp : Defines the entry point for the console application.
//

#include <joint/joint.h>


#include <vector>

using namespace std;


int main(int argc, char** argv)
{
  Joint joint;
  Segment parent("parent", Pose());
  Segment child("child", Pose());

  PID::PIDcoeffs coeffs={1,0,1};

  joint.init("my joint", 0, Joint::ROTATION_3D, coeffs, &parent, &child);
  
  VectorXd state(joint.getDOFnumber());
  state.fill(0);

  VectorXd target(joint.getDOFnumber());
  target.fill(3);

  joint.setState(state);
  joint.setTargetState(target);

  VectorXd result = joint.getTorques();

  cout << "result = " << endl << result << endl;

	 return 0;
}

