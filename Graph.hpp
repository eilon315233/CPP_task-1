// ID: 315233486
// Email: eilonashwal30@gmail.com

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <iostream>
#include <utility>
using namespace std;

namespace ariel
{
    class Graph
    {
       private:
        vector<vector<int>> adjacencyMatrix; // The adjacency matrix of the graph
        bool directed; // Check if the graph is directed

       public:
        void loadGraph(vector<vector<int>> adjMatrix); // Load the graph to the object.

        void printGraph(); // Print the number of vertices and edges in the graph.
        
        size_t getNumberOfEdges(); // Get the number of edges in the graph.

        bool isWeighted(); // Check if the graph is weighted.

        int getWeight(int start, int end); // Get the weight of an edge.

        vector<size_t> getNeighbors(size_t vertex); // Get the neighbors of a vertex.

        bool isDirected(); // Check if the graph is directed.

        size_t getNumberOfVertices(); // Get the number of nodes in the graph.

        vector<vector<int>> getMatrix() const {return adjacencyMatrix;} // Get the adjacency matrix of the graph.(inlined)
        
        bool haveNegativeWeight(); // Check if the graph have negative weight.

        vector<pair<int,pair<int, int>>> getEdges() const; // Get the edges of the graph.(inlined)

        bool isEdge(size_t from, size_t to) { return adjacencyMatrix[from][to] != 0;} // Check if there is an edge between two vertices.(inlined)

        vector<vector<int>> getMatrix() { return adjacencyMatrix;} // Get the adjacency matrix of the graph.(inlined)

    };    
}

#endif // GRAPH_HPP
