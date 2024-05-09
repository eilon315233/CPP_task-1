// ID: 315233486
// Email: eilonashwal30@gmail.com


#include "Graph.hpp"

using namespace ariel;
using namespace std;

// Load the graph to the object.
void Graph::loadGraph(vector<vector<int>> adjMatrix) 
{
    // Check if the graph is empty.
    if (adjMatrix.empty()) { throw invalid_argument("The graph is empty, please enter a new graph."); } 

    // Check if the graph is a square matrix.
    int size = adjMatrix.size();
    for (const auto& row : adjMatrix) 
    {
        if (row.size() != size) {
            throw std::invalid_argument("Invalid graph: The graph is not a square matrix.");
        }
    }

    //load the graph and check if it is directed
    this->adjacencyMatrix = adjMatrix;
    this->directed = isDirected();
}

// Print the number of vertices and edges in the graph and the graph itself.
void Graph::printGraph() 
{
    // Print the graph itself.
    cout << "Graph: " << endl;
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
        cout << "{";
        for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) {
            if (j == adjacencyMatrix[i].size() - 1) {
                cout << adjacencyMatrix[i][j];
            } else {
                cout << adjacencyMatrix[i][j] << ", ";
            }
        }
        cout << "}";
        cout << endl;
    }

    // Print the number of vertices and edges in the graph and if the graph is directed.
    if (!directed)
    { 
        cout << "Graph with " << adjacencyMatrix.size() << " vertices and " << getNumberOfEdges()/2 << " edges," << " is not directed." << endl;
    }
    else
    {
        cout << "Graph with " << adjacencyMatrix.size() << " vertices and " << getNumberOfEdges() << " edges," << "is directed." << endl;
    }    

}

// Check if the graph is directed.
bool Graph::isDirected() 
{
    // Check if the graph is directed by checking if the adjacency matrix is symmetric.
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) 
    {
        for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) 
        {
            if (adjacencyMatrix[i][j] != adjacencyMatrix[j][i]) 
            {
                return true;
            }
        }
    }
    return false;
}

// Get the number of edges in the graph.
size_t Graph::getNumberOfEdges() 
{
    size_t countOfEdge = 0;

    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) 
    {
        for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) 
        {
            if (adjacencyMatrix[i][j] != 0) // Check if there is an edge between two vertices.
            {
                countOfEdge++;
            }
        }
    }
    return countOfEdge;
}

// Get the neighbors of a vertex.
vector<size_t> Graph::getNeighbors(size_t vertex) 
{
    vector<size_t> neighbors;
    for (size_t i = 0; i < adjacencyMatrix[vertex].size(); ++i) 
    {
        if (adjacencyMatrix[vertex][i] != 0) // Check if there is an edge between two vertices.
        {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

// Get the number of nodes in the graph.
size_t Graph::getNumberOfVertices() { return adjacencyMatrix.size();}

// Get the weight of an edge.
int Graph::getWeight(int start, int end) 
{
    // Check if the node index is out of range.
    if (start < 0 || static_cast<size_t>(start) >= adjacencyMatrix.size() || end < 0 || static_cast<size_t>(end) >= adjacencyMatrix.size()) 
    {
        throw std::out_of_range("Node index out of range");
    }
    return adjacencyMatrix[static_cast<size_t>(start)][static_cast<size_t>(end)]; // Return the weight of the edge. 
}

// this function return the edges of the graph
vector<pair<int,pair<int, int>>> Graph::getEdges() const {
    vector<pair<int,pair<int, int>>> edges;
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
        for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) {
            if (adjacencyMatrix[i][j] != 0) {
                edges.push_back({i, {j, adjacencyMatrix[i][j]}});
            }
        }
    }
    return edges;
}
