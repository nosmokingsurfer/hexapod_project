// task_queue_test.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <fstream>

#include <Eigen/Eigen>
#include <control/task_queue/task_queue.h>


using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{


  std::string fileName = "articulated_controls.json";

  TaskQueue controlCommands;
  controlCommands.init(fileName);

  double dt = 0.1;
  for (int i = 0; i < 30; i++)
  {
    cout << "FL:" << endl;
    controlCommands.getControls("FL", i*dt);
    cout << controlCommands.controlSignals["FL"].getCurTargetState(dt*i) << endl;

    cout << "FR:" << endl;
    controlCommands.getControls("FR", i*dt);
    cout << controlCommands.controlSignals["FR"].getCurTargetState(dt*i) << endl;

    cout << "ML:" << endl;
    controlCommands.getControls("ML", i*dt);
    cout << controlCommands.controlSignals["ML"].getCurTargetState(dt*i) << endl;

    cout << "MR:" << endl;
    controlCommands.getControls("MR", i*dt);
    cout << controlCommands.controlSignals["MR"].getCurTargetState(dt*i) << endl;

    cout << "RL:" << endl;
    controlCommands.getControls("RL", i*dt);
    cout << controlCommands.controlSignals["RL"].getCurTargetState(dt*i) << endl;

    cout << "RR:" << endl;
    controlCommands.getControls("RR", i*dt);
    cout << controlCommands.controlSignals["RR"].getCurTargetState(dt*i) << endl;

    cout << "j_1_2:" << endl;
    controlCommands.getControls("j_1_2", i*dt);
    cout << controlCommands.controlSignals["j_1_2"].getCurTargetState(dt*i) << endl;

  }


	 return 0;
}

