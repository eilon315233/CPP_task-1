// ID: 315233486
// Email: eilonashwal30@gmail.com

#include "Algorithms.hpp"
#include <vector>
#include <iostream>
#include "Graph.hpp"
#include <queue>
#include <limits>
#include <stdexcept>
#include <climits>
#include <algorithm>
#include <unordered_set>


using namespace ariel;
using namespace std;

void Algorithms::dfs(Graph graph, size_t vertex, vector<bool>& visited)
{
    // Mark the current vertex as visited
    visited[vertex] = true;

    // Get the neighbors of the current vertex
    vector<size_t> neighbors = graph.getNeighbors(vertex);

    // Recursively visit all unvisited neighbors
    for (size_t neighbor : neighbors)
    {
        if (!visited[neighbor])
        {
            dfs(graph, neighbor, visited);
        }
    }
}

// Check if the graph is connected by check directed or undirected graph and return "1" if connected and "0" if not connected
bool Algorithms::isConnected(Graph graph)
{
    // Check if the graph is empty
    if (graph.getNumberOfEdges() == 0)
    {
        return true; // An empty graph is considered connected
    }

    // Check if the graph is connected

    vector<bool> visited(graph.getNumberOfVertices(), false); // Initialize all vertices as not visited

    dfs(graph, 0, visited); // Perform DFS starting from the first vertex 

    for (bool isVisited : visited)
    {
        if (!isVisited) // If there is a vertex that has not been visited, then the graph is not connected
        {
            return false;
        }
    }
    return true;
}


string Algorithms::isBipartite(Graph grp) {
    size_t numNodes = grp.getNumberOfVertices();
    std::vector<int> colorArr(numNodes, -1);
    std::vector<size_t> setA, setB;

    for (size_t i = 0; i < numNodes; i++) {
        if (colorArr[i] == -1) {
            std::queue<size_t> q;
            q.push(i);
            colorArr[i] = 1;
            while (!q.empty()) {
                size_t u = q.front();
                q.pop();
                std::vector<size_t> neighbors = grp.getNeighbors(u);
                for (size_t v: neighbors) {
                    if (colorArr[v] == -1) {
                        colorArr[v] = 1 - colorArr[u];
                        q.push(v);
                    } else if (colorArr[v] == colorArr[u]) {
//                        std::cout << "0" << std::endl;
                        return "0";
                    }
                }
            }
        }
    }

    for (size_t i = 0; i < numNodes; i++) {
        if (colorArr[i] == 1) {
            setA.push_back(i);
        } else {
            setB.push_back(i);
        }
    }

    std::string result = "The graph is bipartite: A={";
    for (size_t i = 0; i < setA.size(); i++) {
        result += std::to_string(setA[i]);
        if (i != setA.size() - 1) {
            result += ", ";
        }
    }
    result += "}, B={";
    for (size_t i = 0; i < setB.size(); i++) {
        result += std::to_string(setB[i]);
        if (i != setB.size() - 1) {
            result += ", ";
        }
    }
    result += "}";
//    std::cout << result << std::endl;
    return result;
}


string Algorithms::shortestPath(Graph grp, size_t start, size_t end) {
    size_t numNodes = grp.getNumberOfVertices();
    vector<int> dist(numNodes, numeric_limits<int>::max());
    vector<size_t> pred(numNodes, numeric_limits<size_t>::max());
    dist[start] = 0;

    // Relax all edges |V| - 1 times
    bool updated = false;
    for (size_t i = 0; i < numNodes - 1; i++) {
        updated = false;
        for (auto edge : grp.getEdges()) {
            size_t u = static_cast<size_t>(edge.first);
            size_t v = static_cast<size_t>(edge.second.first);
            int weight = edge.second.second;
            if (dist[u] != numeric_limits<int>::max() && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pred[v] = u;
                updated = true;
            }
        }
        if (!updated) break; // If no update is made, stop early
    }

    // Check for negative-weight cycles in the last iteration
    if (updated) {
        for (auto edge : grp.getEdges()) {
            size_t u = static_cast<size_t>(edge.first);
            size_t v = static_cast<size_t>(edge.second.first);
            int weight = edge.second.second;
            if (dist[u] != numeric_limits<int>::max() && dist[u] + weight < dist[v]&&pred[u]!=v) {
                throw runtime_error("Graph contains a negative-weight cycle");
            }
        }
    }

    // Construct the path from start to end
    if (dist[end] == numeric_limits<int>::max()) {
        return "-1";  // No path found
    }

    vector<size_t> path;
    for (size_t at = end; at != start; at = pred[at]) {
        if (at == numeric_limits<size_t>::max()) {
            return "-1";  // No valid path exists
        }
        path.push_back(at);
    }
    path.push_back(start); // Add the start node at the end

    reverse(path.begin(), path.end());

    // Create a string representation of the path
    string pathStr = to_string(path[0]);
    for (size_t i = 1; i < path.size(); ++i) {
        pathStr += "->" + to_string(path[i]);
    }

    return pathStr;
}


bool Algorithms::isCyclicUtil(size_t vertex, vector<bool>& visited, vector<bool>& recStack, vector<size_t >& parent,Graph graph, vector<size_t>& cycle){
visited[vertex] = true;
recStack[vertex] = true;
bool isDirected = graph.isDirected();

size_t numNodes = graph.getNumberOfVertices();
for (size_t i = 0; i < numNodes; i++) {
    if (graph.isEdge(vertex, i)) {  // Check if there is an edge from v to i
        if (!visited[i]) {
            parent[i] = vertex;
            if (isCyclicUtil(i, visited, recStack, parent, graph, cycle)) {
                return true;
            }
        } else if ((isDirected && recStack[i]) || (!isDirected && recStack[i] && parent[vertex] != i)) {
            // If a cycle is detected, trace back to print the cycle
            cycle.push_back(i);
            for (size_t p = vertex; p != i; p = parent[p]) {
                cycle.push_back(p);
            }
            cycle.push_back(i);  // Complete the cycle by adding the start node again
            std::reverse(cycle.begin(), cycle.end());

            // Print the cycle path
            std::cout << "The cycle is: ";
            for (size_t j = 0; j < cycle.size(); j++) {
                std::cout << cycle[j];
                if (j < cycle.size() - 1) std::cout << "->";
            }
            std::cout << std::endl;
            return true;
        }
    }
}

recStack[vertex] = false;
return false;
}

bool Algorithms::isContainsCycle(Graph graph)
{
    
    size_t numNodes = graph.getNumberOfVertices(); // Get the number of nodes in the graph

    vector<bool> visited(numNodes, false); // Initialize all nodes as not visited

    vector<bool> recStack(numNodes, false); // Initialize all nodes as not part of the recursion stack

    vector<size_t> parent(numNodes, SIZE_MAX); // Initialize all parents as not visited

    vector<size_t> cycle; // Initialize the cycle vector

    for (size_t i = 0; i < numNodes; i++) {
        if (!visited[i]) {
            if (isCyclicUtil(i, visited, recStack, parent, graph, cycle)) {
                return true;
            }
        }
    }
    return false;
}

bool Algorithms::dfsCycle(Graph graph, size_t vertex, vector<bool>& visited, vector<size_t>& path)
{
    visited[vertex] = true;
    path.push_back(vertex);

    vector<size_t> neighbors = graph.getNeighbors(vertex);
    for (size_t neighbor : neighbors)
    {
        if (!visited[neighbor])
        {
            if (dfsCycle(graph, neighbor, visited, path))
            {
                return true;
            }
        }
        else if (find(path.begin(), path.end(), neighbor) != path.end())
        {
            // Found a back edge, indicating a cycle
            return true;
        }
    }
    
    path.pop_back(); // Remove the current vertex from the path
    return false;
}

string Algorithms::negativeCycle(Graph graph)
{
    size_t numNodes = graph.getNumberOfVertices();
    std::vector<int> dist(numNodes, INT_MAX);
    std::vector<int> parent(numNodes, -1);
    size_t source = 0; // Let's take 0 as the source node
    dist[source] = 0;

    // Relax all edges V - 1 times
    for (size_t i = 0; i < numNodes - 1; i++) {
        for (size_t u = 0; u < numNodes; u++) {
            std::vector<size_t> neighbors = graph.getNeighbors(u);
            for (size_t v: neighbors) {
                int weight = graph.getWeight(u, v);
                if (dist[u] != INT_MAX && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                }
            }
        }
    }

    // Check for negative-weight cycles
    for (size_t u = 0; u < numNodes; u++) {
        std::vector<size_t> neighbors = graph.getNeighbors(u);
        for (size_t v: neighbors) {
            int weight = graph.getWeight(u, v);
            if (dist[u] != INT_MAX && dist[u] + weight < dist[v]) {
                // Negative cycle found, construct the cycle path
                std::vector<int> cycle;
                for (size_t v = u;; v = static_cast<size_t>(parent[v])) {
                    cycle.push_back(v);
                    if (v == u && cycle.size() > 1)
                        break;
                }
                std::reverse(cycle.begin(), cycle.end());

                std::string cycleStr = "The cycle is: ";
                for (size_t i = 0; i < cycle.size(); ++i) {
                    cycleStr += std::to_string(cycle[i]);
                    if (i != cycle.size() - 1) {
                        cycleStr += "->";
                    }
                }
                return cycleStr;
            }
        }
    }

    return "0";
}


