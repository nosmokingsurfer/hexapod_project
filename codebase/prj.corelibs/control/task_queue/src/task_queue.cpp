#include <control/task_queue/task_queue.h>

#include <fstream>

TaskQueue::TaskQueue()
{}

TaskQueue::~TaskQueue()
{}

bool TaskQueue::init(const std::string scriptname)
{
  std::ifstream input(scriptname);
  if (input.is_open()) {}
  else {
	  std::cout << "Json library is missing";
	  return false;
  }
  input >> this->controlScript;
  input.close();
 
  return true;
}

VectorXd TaskQueue::getControls(const std::string objectName, const double time)
{
  VectorXd result;

  int frameNumber = int(time);

  for (auto it = controlScript.begin(); it!=controlScript.end(); ++it)
  {
    if ((*it)["start_time"].asDouble() == frameNumber*1.0)
    {
      if(!(*it)[objectName].empty())
      {
        Json::Value object = (*it)[objectName];
        
        Json::Value arr(Json::arrayValue);
        arr = object["from"];

        VectorXd from(arr.size());
        from.fill(0);
        for (int i = 0; i < static_cast<int>(arr.size()); i++)
        {
          from[i] = arr[i].asDouble();
        }

        arr = object["to"];
        VectorXd to(arr.size());
        to.fill(0);
        for (int i = 0; i < static_cast<int>(arr.size()); i++)
        {
          to[i] = arr[i].asDouble();
        }
        double duration = object["duration"].asDouble();
        double start_time = object["start_time"].asDouble();
        controlSignals[objectName] = LinearPlayer(from, to, duration, start_time);
        result = controlSignals[objectName].getCurTargetState(time);
        return result;
      }
    }
  }
  result = controlSignals[objectName].getCurTargetState(time);
  return result;
}

Json::Value TaskQueue::setLeg(const Vector3d& from, const Vector3d& to, const double duration, const double start_time)
{
  Json::Value result;

  Json::Value temp(Json::arrayValue);
  for (auto i = 0; i < 3; i++)
  {
    temp.append(from[i]);
  }
  result["from"] = temp;

  temp.clear();
  for (auto i = 0; i < 3; i++)
  {
    temp.append(to[i]);
  }
  result["to"] = temp;

  result["duration"] = duration;
  result["start_time"] = start_time;

  return result;
}
