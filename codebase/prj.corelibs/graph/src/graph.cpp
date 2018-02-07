#include <graph/graph.h>

Graph::Graph()
{}

Graph::~Graph()
{}


void Graph::addVertex(const string& name, const Pose& pose)
{
  vmap::iterator itr = g.find(name);
  if (itr == g.end())
  {
    Vertex *v;
    v = new Vertex(name, pose);
    v->used = false;
    g[name] = v;
    return;
  }
  cout << "\nVertex already exists!";
}

void Graph::addEdge(const string& from, const string& to, const Pose& pose)
{
  Vertex *f = (g.find(from)->second);
  Vertex *t = (g.find(to)->second);
  pair<Pose, Vertex *> edge = make_pair(pose, t);
  f->adj.push_back(edge);
}

void Graph::addBidirectional(const string& from, const string& to, const Pose& pose)
{
  Pose temp(pose);
  addEdge(from,to, temp);
  addEdge(to,from, temp.inverse());
}

void Graph::DFS(const string name)
{
  g.find(name)->second->used = true;
  
  for (vector<Vertex::ve>::iterator i=g.find(name)->second->adj.begin(); i != g.find(name)->second->adj.end(); ++i)
    if (!(*i).second->used)
    {
      cout << (*i).second->name << endl;
      this->DFS((*i).second->name);
    }
}


Pose Graph::DFS(const string from, const string to, Pose pose)
{
  g.find(from)->second->used = true;

  if (from == to)
  {
    return g.find(from)->second->pose*pose;
  }

  Vertex& curVertex = *g.find(from)->second;

  for (auto it = curVertex.adj.begin(); it != curVertex.adj.end(); ++it)
  {
    if (!it->second->used)
    {
      Pose result = DFS(it->second->name, to, it->first*curVertex.pose*pose);
      cout << it->second->name << endl;
      return result;
    }
  }

  return Pose();
}

void Graph::setNotUsed()
{
  for (auto it = g.begin(); it!= g.end(); ++it)
  {
    it->second->used = false;
  }
}
