// single_leg_test.cpp : Defines the entry point for the console application.
//

#include <step/step.h>

#include <vector>

using namespace std;


int main(int argc, char** argv)
{
  Step simpleStep;

  simpleStep.init(1, 0.5, 0, 2);

  double time = 0;
  double dt = 0.01;

  for (int i = 0; i < 1000; i++)
  {
    simpleStep.getStepNumber(time);
    simpleStep.getStepPhase(time);
    simpleStep.getTargetPoint(0, 0, time);
    time +=dt;
  }

	 return 0;
}

