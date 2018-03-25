// task_queue_test.cpp : Defines the entry point for the console application.
//

#include "../thirdparty/jsoncpp_headers/json/json.h"

#include <iostream>
#include <fstream>

#include <Eigen/Eigen>
#include <control/task_queue/task_queue.h>


using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  

#if 0
  std::string filename = "controlTasks.json";
  
  Json::Value root(Json::arrayValue);

  for (auto n = 0; n < 10; n++)
  {
    Json::Value shot;

    shot["case"]["number"] = n;
    
    Vector3d A(0,0,0);
    Vector3d B(1,2,3);
    

    Json::Value temp(Json::arrayValue);
    for (auto i = 0; i < A.size(); i++)
    {
      temp.append(A[i]);
    }

    shot["case"]["from"] = temp;


    temp.clear();
    for (auto i =0; i < B.size(); i++)
    {
      temp.append(B[i]);
    }

    shot["case"]["to"] = temp;


    shot["case"]["start_time"] = 1.0;
    shot["case"]["duration"] = 1.0;

    root.append(shot);
  }

  Json::StyledWriter writer;
  std::ofstream output(filename);

  output << writer.write(root);

  output.close();

  std::ifstream input(filename);

  Json::Value controlSeq;

  input >> controlSeq;

  for (auto it = controlSeq.begin(); it != controlSeq.end(); ++it)
  {
    
    if ((*it)["case"]["number"].asInt() == 3)
    {
      cout << *it << endl;
    }
  }
#endif

 std::string fileName = "articulated_controls.json";
  //Json::Value root(Json::arrayValue);
  //Json::Value frame;
  //frame["start_time"] = 0;
  //frame["duration"] = 1.0;
  //frame["FL"] = TaskQueue::setLeg(Vector3d(-0.5, 0.5, 0.0), Vector3d(-0.5,0.5,0.1), 1.0,0);
  //frame["FR"] = TaskQueue::setLeg(Vector3d(0.5, 0.5, 0.0),  Vector3d(0.5,0.5,0.1), 1.0,0);
  //
  //root.append(frame);
  //
  //frame["start_time"] = 1.0;
  //
  //frame["FL"] = TaskQueue::setLeg(Vector3d(-0.5, 0.5, 0.1), Vector3d(-0.5,1.01,0.1), 1.0,1.0);
  //frame["FR"] = TaskQueue::setLeg(Vector3d(0.5, 0.5, 0.1),  Vector3d(0.5,1.01,0.1), 1.0,1.0);
  //
  //root.append(frame);
  //
  //std::ofstream output(fileName);
  //Json::StyledWriter writer;
  //output << writer.write(root); 
  //output.close();


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

