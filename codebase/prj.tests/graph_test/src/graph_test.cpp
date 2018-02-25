// trajectory_test.cpp : Defines the entry point for the console application.
//

#include <graph/graph.h>

#include <vector>

using namespace std;


int main(int argc, char** argv)
{

  Graph myGraph;

  myGraph.addVertex("b1", Pose());
  myGraph.addVertex("b2", Pose());
  myGraph.addVertex("b3", Pose());
  myGraph.addVertex("b4", Pose());
  myGraph.addVertex("b5", Pose());

  myGraph.addBidirectional("b1", "b2", Pose(Vector3d(EIGEN_PI/2, 0, 0), Vector3d(10,0,0)));
  myGraph.addBidirectional("b2", "b3", Pose(Vector3d(EIGEN_PI/2, 0, 0), Vector3d(-10,0,0)));
  myGraph.addBidirectional("b3", "b4", Pose(Vector3d(EIGEN_PI/2, 0, 0), Vector3d(10,0,0)));
  myGraph.addBidirectional("b4", "b5", Pose(Vector3d(EIGEN_PI/2, 0, 0), Vector3d(-10,0,0)));

  myGraph.deleteEdge("b2", "b3");


  Pose result = myGraph.DFS("b5", "b1"); // should be E matrix here

	 return 0;
}

