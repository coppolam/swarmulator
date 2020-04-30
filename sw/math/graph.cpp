#include "graph.h"

/**
 * Add Edge to connect v and w
 */
void Graph::addEdge(int v, int w)
{
  adj[v].push_back(w);
  adj[w].push_back(v);
}


/**
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

/**
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

void Graph::DFS(int v, bool visited[])
{
  // Mark the current node as visited and print it
  visited[v] = true;

  // Recur for all the vertices
  // adjacent to this vertex
  list<int>::iterator i;
  for (i = adj[v].begin(); i != adj[v].end(); ++i)
    if (!visited[*i]) {
      DFS(*i, visited);
    }
}

// Method to print connected components in an
// undirected graph
uint Graph::connectedComponents()
{
  // Mark all the vertices as not visited
  bool *visited = new bool[V];
  for (int v = 0; v < V; v++) {
    visited[v] = false;
  }
  uint count = 0;
  for (int v = 0; v < V; v++) {
    if (visited[v] == false) {
      // print all reachable vertices
      // from v
      DFS(v, visited);
      count++;
    }
  }
  delete[] visited;

  return count;
}

/**
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
