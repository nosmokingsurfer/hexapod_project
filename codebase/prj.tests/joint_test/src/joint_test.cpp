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


  joint.init(0, Joint::JOINT_TYPE::ROTATION_1D, PID::PIDcoeffs(), &parent, &child);
  
	 return 0;
}

