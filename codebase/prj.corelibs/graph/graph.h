///
///@file dummy/dummy.h
///@authors Panchenko A.V.
///@brief Dummy class for Core  libs
///


#pragma once
#ifndef GRAPH_H
#define GRAPH_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <map>
#include <iostream>

#include <pose/pose.h>

using namespace std;
using namespace Eigen;


struct Vertex {

  vector<Vertex*> adj; // edges
  bool used; // was this vertex visited
  string name; // vertex name
  Vertex(const string s): name(s) {}
  //payload goes here
  Pose vertexPose;
};


class Graph{
public: 
  Graph();
  ~Graph();

  typedef map<string, Vertex *> vmap; 
  vmap g;



  void addVertex(const string&);
  void addEdge(const string& from, const string& to); //Pose?

  void DFS(const string name);

  void setNotUsed();
};

#endif