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

/*
 * Defines a graph structure that can be used to assess the topology of the swarm
 */
class Graph
{
private:
  int V;
  list<int> *adj;

public:
  /*
   * Constructor instatiates a list of all connections in the graph.
   */
  Graph(int V)
  {
    this->V = V;
    adj = new list<int>[V];
  }

  /*
   * Function to add an edge in the topology graph between agent v and w
   */
  void addEdge(int v, int w);

  /*
   * Breadth-First Search
   */
  void BFS(int s, bool visited[]);

  /*
   * Returns the transpose of the graph
   */
  Graph getTranspose();

  /*
   * Checks that the topology is connected.
   * Returns true if connected, false if not connected
   */
  bool isConnected();
};

#endif /* GRAPH_H */