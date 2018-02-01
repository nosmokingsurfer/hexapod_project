#include <graph/graph.h>

Graph::Graph()
{}

Graph::~Graph()
{}

void Graph::addvertex(const string& name)
{
  vmap::iterator itr = g.find(name);
  if (itr == g.end())
  {
    Vertex *v;
    v = new Vertex(name);
    v->used = false;
    g[name] = v;
    return;
  }
  cout << "\nVertex already exists!";
}

void Graph::addedge(const string& from, const string& to, double cost)
{
  Vertex *f = (g.find(from)->second);
  Vertex *t = (g.find(to)->second);
  pair<int, Vertex *> edge = make_pair(cost, t);
  f->adj.push_back(edge);
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

  for (auto it = g.find(from)->second->adj.begin(); it != g.find(from)->second->adj.end(); ++it)
  {
    if (!it->second->used)
    {
      cout << it->second->name << endl;
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
