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

  typedef pair<Pose, Vertex*> ve;
  vector<ve> adj;
  bool used;
  string name;
  Pose pose;
  Vertex(const string s, const Pose p) : name(s), pose(p) {}
};





class Graph{
public: 
  Graph();
  ~Graph();

  typedef map<string, Vertex *> vmap; // определяем новый тип данных
  vmap g; //вершины графа хранятся здесь



  void addVertex(const string&, const Pose& pose); //добавляем вершину
  void addEdge(const string& from, const string& to, const Pose& pose); //добавляем ребро
  void addBidirectional(const string& from, const string& to, const Pose& pose); //добавляем двунаправленное ребро

  void DFS(const string name); //обход графа из этой вершины

  Pose DFS(const string from, const string to, Pose = Pose()); // находим Pose из вершины from в вершину to

  void setNotUsed(); // выставляем вершине не посещенными

};

#endif