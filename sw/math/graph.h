/*
* C++ Program to Check whether Undirected Graph is Connected using BFS
* Code extracted from Sanfoundry Global Education & Learning Series
* http://www.sanfoundry.com/cpp-program-check-undirected-graph-connected-bfs/
*/
#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <list>
#include <queue>

using namespace std;

class Graph
{
private:
  int V;
  list<int> *adj;

public:
  Graph(int V)
  {
    this->V = V;
    adj = new list<int>[V];
  }

  void addEdge(int v, int w);
  void BFS(int s, bool visited[]);

  Graph getTranspose();
  bool isConnected();
};

#endif /* GRAPH_H */