#include "graph.h"

/*
 * Add Edge to connect v and w
 */
void Graph::addEdge(int v, int w)
{
  adj[v].push_back(w);
  adj[w].push_back(v);
}

/*
 *  A recursive function to print BFS starting from s
 */
void Graph::BFS(int s, bool visited[])
{
  list<int> q;
  list<int>::iterator i;
  visited[s] = true;
  q.push_back(s);
  while (!q.empty()) {
    s = q.front();
    q.pop_front();
    for (i = adj[s].begin(); i != adj[s].end(); ++i) {
      if (!visited[*i]) {
        visited[*i] = true;
        q.push_back(*i);
      }
    }
  }
}

/*
 * Function that returns reverse (or transpose) of this graph
 */
Graph Graph::getTranspose()
{
  Graph g(V);
  for (int v = 0; v < V; v++) {
    list<int>::iterator i;
    for (i = adj[v].begin(); i != adj[v].end(); ++i) {
      g.adj[*i].push_back(v);
    }
  }
  return g;
}

/*
 * Check if Graph is Connected
 */
bool Graph::isConnected()
{
  bool visited[V];
  for (int i = 0; i < V; i++) {
    visited[i] = false;
  }

  BFS(0, visited);
  for (int i = 0; i < V; i++)
    if (visited[i] == false) {
      return false;
    }

  Graph gr = getTranspose();

  for (int i = 0; i < V; i++) {
    visited[i] = false;
  }

  gr.BFS(0, visited);
  for (int i = 0; i < V; i++)
    if (visited[i] == false) {
      return false;
    }

  return true;
}