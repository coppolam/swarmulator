
#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <list>
#include <queue>

using namespace std;

/**
 * @brief Defines a graph structure that can be used to assess the topology of the swarm
 *
 * Code in part extracted from Sanfoundry Global Education & Learning Series
 * http://www.sanfoundry.com/cpp-program-check-undirected-graph-connected-bfs/
 */
class Graph
{
private:
  int V;
  list<int> *adj;

public:
  /**
   * Constructor instatiates a list of all connections in the graph.
   *
   * @param V Total number of vertices
   */
  Graph(int V)
  {
    this->V = V;
    adj = new list<int>[V];
  }

  /**
   * Function to add an edge in the topology graph between node v and w
   *
   * @param v ID of node v
   * @param w ID of node w
   */
  void addEdge(int v, int w);

  /**
   * Returns the transpose of the graph
   */
  Graph getTranspose();

  /**
   * Breadth-First Search (BFS) implementation. See how it is used in graph::isConnected()
   *
   * @param s first node to be visited (use s=0)
   * @param visited visited nodes during BFS
   */
  void BFS(int s, bool visited[]);

  /**
   * Depth-First Search
   *
   * @param v
   * @param visited
   */
  void DFS(int v, bool visited[]);

  /**
   * @brief Checks that the topology is connected.
   *
   * Check whether the undirected graph is connected using BFS
   * Returns true if connected, false if not connected
   */
  bool isConnected();

  /**
   * Counts the number of connected robot clusters
   *
   * @return uint
   */
  uint connectedComponents();
};

#endif /* GRAPH_H */
