// ID: 315233486
// Email: eilonashwal30@gmail.com


#include "Graph.hpp"
#include <iostream>

using namespace ariel;
using namespace std;

void Graph::loadGraph(vector<vector<int>> adjMatrix) {
    if (adjMatrix.empty()) {
        throw invalid_argument("The graph is empty, please enter a new graph.");
    }

    int size = adjMatrix.size();
    for (const auto& row : adjMatrix) {
        if (row.size() != size) {
            throw std::invalid_argument("Invalid graph: The graph is not a square matrix.");
        }
    }

    this->adjacencyMatrix = adjMatrix;
    this->directed = isDirected();
}

void Graph::printGraph() {

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

    cout << "Graph with " << adjacencyMatrix.size() << " vertices and " << getNumberOfEdges() << " edges." << endl;
}

bool Graph::isDirected() {
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
        for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) {
            if (adjacencyMatrix[i][j] != 0 && adjacencyMatrix[j][i] == 0) {
                return true;
            }
        }
    }
    return false;
}

size_t Graph::getNumberOfEdges() {
    size_t countOfEdge = 0;

    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
        for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) {
            if (adjacencyMatrix[i][j] != 0) {
                countOfEdge++;
            }
        }
    }
    return countOfEdge;
}

bool Graph::isWeighted() {
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
        for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) {
            if (adjacencyMatrix[i][j] != 0) {
                return true;
            }
        }
    }
    return false;
}

bool Graph::haveNegativeWeight() {
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
        for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) {
            if (adjacencyMatrix[i][j] < 0) {
                return true;
            }
        }
    }
    return false;
}

vector<size_t> Graph::getNeighbors(size_t vertex) {
    vector<size_t> neighbors;
    for (size_t i = 0; i < adjacencyMatrix[vertex].size(); ++i) {
        if (adjacencyMatrix[vertex][i] != 0) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

size_t Graph::getNumberOfVertices() {
    return adjacencyMatrix.size();
}

int Graph::getWeight(int start, int end) {
    if (start < 0 || static_cast<size_t>(start) >= adjacencyMatrix.size() || end < 0 || static_cast<size_t>(end) >= adjacencyMatrix.size()) {
        throw std::out_of_range("Node index out of range");
    }
    return adjacencyMatrix[static_cast<size_t>(start)][static_cast<size_t>(end)];
}

vector<pair<int,pair<int, int>>> Graph::getEdges() const {
    std::vector<std::pair<int, std::pair<int, int>>> edges;
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
        for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) {
            if (adjacencyMatrix[i][j] != 0) {
                edges.push_back({i, {j, adjacencyMatrix[i][j]}});
            }
        }
    }
    return edges;
}
