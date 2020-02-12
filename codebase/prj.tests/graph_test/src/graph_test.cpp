// trajectory_test.cpp : Defines the entry point for the console application.
//
<<<<<<< HEAD
/*
=======


>>>>>>> moving from laptop
#include <graph/graph.h>
#include <iostream>
#include <vector>

using namespace std;

const double precision = 0.000000001;

Vector3d TestNewApproach(void)
{
    Graph myGraph;

    myGraph.addVertex("b1", Pose());
    myGraph.addVertex("b2", Pose());

    myGraph.addBidirectional("b1", "b2", Pose(Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
    Vector3d result1 = myGraph.getCoord(Vector3d(1, 1, 1), "b1", "b2");
    Pose pose = myGraph.DFS("b1", "b2", Pose(Vector3d(0,0,0))) ;
    return pose.T*Vector3d(1, 1, 1);
    
}

bool TestZrotation(void)
{
    Graph myGraph;

    myGraph.addVertex("b1", Pose());
    myGraph.addVertex("b2", Pose());

    myGraph.addBidirectional("b1", "b2", Pose(Vector3d(EIGEN_PI / 2, 0, 0), Vector3d(10, 0, 0)));

    Vector3d result1 = myGraph.getCoord(Vector3d(1, 2, 3), "b1", "b2");
    Vector3d result2 = myGraph.getCoord(Vector3d(0, 0, 0), "b1", "b2");
    Vector3d result3 = myGraph.getCoord(Vector3d(-1, -2, -3), "b1", "b2");
    Vector3d CorrectAnswer1(2, 9, 3);
    bool answer(0);
    if ((result1 - CorrectAnswer1).norm() > precision)
    {
        answer = false;
    }
   Vector3d CorrectAnswer2(0, 10, 0);
    if ((result2 - CorrectAnswer2).norm() > precision)
    {
        answer = false;
    }
    Vector3d CorrectAnswer3(-2, 11, -3);
    if ((result3 - CorrectAnswer3).norm() > precision)
    {
        answer = false;
    }
    return answer;
    
}
bool TestDoubleRotation(void)
{
    Graph myGraph;
    myGraph.addVertex("b1", Pose());
    myGraph.addVertex("b2", Pose());

    myGraph.addBidirectional("b1", "b2", Pose(Vector3d(EIGEN_PI / 2, 3 * EIGEN_PI / 2, 0), Vector3d(0, 0, 0)));

    Vector3d result1 = myGraph.getCoord(Vector3d(1, 2, 3), "b1", "b2");
    Vector3d result2 = myGraph.getCoord(Vector3d(-1, -2, -3), "b1", "b2");
    bool answer(0);
    Vector3d CorrectAnswer1(2, -3, -1);
    if ((result1 - CorrectAnswer1).norm() > precision)
    {
        answer = false;
    }

    Vector3d CorrectAnswer2(-2, 3, 1);
    if ((result2 - CorrectAnswer2).norm() > precision)
    {
        answer = false;
    }
    return answer;
}
bool TestDegRotation(void)
{
    Graph myGraph;
    myGraph.addVertex("b1", Pose());
    myGraph.addVertex("b2", Pose());

    myGraph.addBidirectional("b1", "b2", Pose(Vector3d(2 * EIGEN_PI , 0, 0), Vector3d(0, 0, 0)));
    Vector3d result1 = myGraph.getCoord(Vector3d(1, 2, 3), "b1", "b2");
    Vector3d result2 = myGraph.getCoord(Vector3d(-1, -2, -3), "b1", "b2");
    bool answer(true);
    if (result1 != Vector3d(1, 2, 3)) answer = false;
    if (result2 != Vector3d(-1, -2, -3)) answer = false;
    return answer;
}
Vector3d Rotation(void)
{
    Graph myGraph;
    myGraph.addVertex("b1", Pose());
    myGraph.addVertex("b2", Pose());

    myGraph.addBidirectional("b1", "b2", Pose(Vector3d(EIGEN_PI / 2, EIGEN_PI / 3, EIGEN_PI / 4), Vector3d(0, 0, 0)));
    Vector3d result = myGraph.getCoord(Vector3d(1, 2, 3), "b1", "b2");
    return result;
}
Vector3d FinalTests(void)
{
    Graph myGraph;
    myGraph.addVertex("b1", Pose());
    myGraph.addVertex("b2", Pose());

    myGraph.addBidirectional("b1", "b2", Pose(Vector3d(2 * EIGEN_PI, EIGEN_PI / 2, 3 * EIGEN_PI / 2), Vector3d(0, 0, 0)));

int main(int argc, char** argv)
{
    bool allTestsCorrect = true;
  
    allTestsCorrect &= TestZrotation();
    allTestsCorrect &= TestDoubleRotation();
    allTestsCorrect &= TestDegRotation();
    cout << TestNewApproach() << endl;
    cin.get();
  return 0;
}*/

