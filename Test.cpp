// ID: 315233486
// Email: eilonashwal30@gmail.com

#include <iostream>
#include <vector>

#include "doctest.h"
#include "Algorithms.hpp"
#include "Graph.hpp"
using namespace std;

TEST_CASE("Test isConnected")
{

    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isConnected(g) == true);

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isConnected(g) == false);

    vector<vector<int>> graph4 = {
        {0, 1, 0},
        {1, 0, 0},
        {0, 0, 0}};
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::isConnected(g) == false);

    vector<vector<int>> graph5 = {
        {0, 1, 0},
        {0, 0, 1},
        {1, 0, 0}};
    g.loadGraph(graph5);
    CHECK(ariel::Algorithms::isConnected(g) == true);
}

TEST_CASE("Test ShortestPath")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2");

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 4) == "-1");

    vector<vector<int>> graph4 = {
        {0, 1, 0},
        {1, 0, 0},
        {0, 0, 0}};
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "-1");

    vector<vector<int>> graph6 = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph6);
    CHECK(ariel::Algorithms::shortestPath(g, 3, 2) == "-1");
}
TEST_CASE("Test shortestPath with disconnected graph")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
            {0, 1, 0, 0, 0},
            {1, 0, 0, 0, 0},
            {0, 0, 0, 1, 0},
            {0, 0, 1, 0, 0},
            {0, 0, 0, 0, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 3) == "-1");
}

TEST_CASE("Test shortestPath with negative cycle")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
            {0, 1, 0, 0, 0},
            {1, 0, -3, 0, 0},
            {0, -8, 0, 1, 0},
            {0, 0, 2, 0, 5},
            {0, 0, 0, 5, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 4) == "The graph contains a negative cycle");
    }

TEST_CASE("Test isContainsCycle")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isContainsCycle(g) == false);

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isContainsCycle(g) == true);

    vector<vector<int>> graph4 = {
        {0, 1, 0},
        {1, 0, 0},
        {0, 0, 0}};
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::isContainsCycle(g) == false);

    vector<vector<int>> graph5 = {
        {0, 1, 0},
        {0, 0, 1},
        {1, 0, 0}};
    g.loadGraph(graph5);
    CHECK(ariel::Algorithms::isContainsCycle(g) == true);

    vector<vector<int>> graph6 = {
        {0, 1, 0},
        {0, 0, 0},
        {1, 0, 0}};
    g.loadGraph(graph6);
    CHECK(ariel::Algorithms::isContainsCycle(g) == false);

    vector<vector<int>> graph7 = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph7);
    CHECK(ariel::Algorithms::isContainsCycle(g) == false);

    vector<vector<int>> graph8 = {
        {0, 1, 1},
        {1, 0, 1},
        {1, 1, 0}};
    g.loadGraph(graph8);
    CHECK(ariel::Algorithms::isContainsCycle(g) == true);
}
TEST_CASE("Test isBipartite")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0, 2}, B={1}");

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isBipartite(g) == "0");

    vector<vector<int>> graph3 = {
        {0, 1, 0, 0, 0},
        {1, 0, 3, 0, 0},
        {0, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, 5, 0}};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0, 2, 4}, B={1, 3}");

    vector<vector<int>> graph5 = {
        {0, 1, 1},
        {1, 0, 1},
        {1, 1, 0}};
    g.loadGraph(graph5);
    CHECK(ariel::Algorithms::isBipartite(g) == "0"); 
}

TEST_CASE("Test invalid graph")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 2, 0},
        {1, 0, 3, 0},
        {2, 3, 0, 4},
        {0, 0, 4, 0},
        {0, 0, 0, 5}};
    CHECK_THROWS(g.loadGraph(graph));

    vector<vector<int>> graph2 = {
        {0, 1, 2},
        {1, 0, 3},
        {2, 3, 0},
        {0, 0, 4},
        {0, 0, 5}};
    CHECK_THROWS(g.loadGraph(graph2));

    vector<vector<int>> graph3 = {
        {0, 1, 2, 2, 2, 2, 2},
        {2, 3, 4, 5, 6, 0, 7},
        {2, 3, 4, 5, 6, 7, 0}};
    CHECK_THROWS(g.loadGraph(graph3));
}

TEST_CASE("Test loadGraph with valid graph")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
            {0, 1, 0, 0, 0},
            {1, 0, 3, 0, 0},
            {0, 3, 0, 4, 0},
            {0, 0, 4, 0, 5},
            {0, 0, 0, 5, 0}};
    g.loadGraph(graph);
    CHECK(g.getNumberOfVertices() == 5);
}

TEST_CASE("Test loadGraph with empty graph")
{
    ariel::Graph g;
    vector<vector<int>> graph = {};
    CHECK_THROWS_AS(g.loadGraph(graph), std::invalid_argument);
}

TEST_CASE("Test loadGraph with non-square matrix")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
            {0, 1, 0},
            {1, 0, 3, 0},
            {0, 3, 0, 4, 0},
            {0, 0, 4, 0, 5},
            {0, 0, 0, 5, 0}};
    CHECK_THROWS_AS(g.loadGraph(graph), std::invalid_argument);
}

TEST_CASE("Test DFS")
{

    ariel::Graph g;
    vector<vector<int>> graph = {
            {0, 1, 0, 0, 0},
            {1, 0, 3, 0, 0},
            {0, 3, 0, 4, 0},
            {0, 0, 4, 0, 5},
            {0, 0, 0, 5, 0}};
    g.loadGraph(graph);

    std::vector<bool> visited(g.getNumberOfVertices(), false);
    ariel::Algorithms::dfs(g, 0, visited);

    // All vertices should be visited
    for (bool v : visited) {
        CHECK(v == true);
    }
}

TEST_CASE("Test DFS with disconnected graph")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
            {0, 1, 0, 0, 0},
            {1, 0, 0, 0, 0},
            {0, 0, 0, 1, 0},
            {0, 0, 1, 0, 0},
            {0, 0, 0, 0, 0}};
    g.loadGraph(graph);

    std::vector<bool> visited(g.getNumberOfVertices(), false);
    ariel::Algorithms::dfs(g, 0, visited);

    // Only vertices 0 and 1 should be visited
    CHECK(visited[0] == true);
    CHECK(visited[1] == true);
    CHECK(visited[2] == false);
    CHECK(visited[3] == false);
    CHECK(visited[4] == false);
}

TEST_CASE("Test negativeCycle with no negative cycle")
{
    ariel::Graph g;
     vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::negativeCycle(g) == "the graph does not contain a negative cycle");

    vector<vector<int>> graph2 = {
            {0, 1, 0, 0, 0},
            {1, 0, 3, 0, 0},
            {0, 3, 0, 4, 0},
            {0, 0, 4, 0, 5},
            {0, 0, 0, 5, 0}};
    g.loadGraph(graph2);

    CHECK(ariel::Algorithms::negativeCycle(g) == "the graph does not contain a negative cycle");
}

TEST_CASE("Test negativeCycle with negative cycle")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
            {0, 1, 0, 0, 0},
            {1, 0, -3, 0, 0},
            {0, -3, 0, 4, 0},
            {0, 0, 4, 0, 5},
            {0, 0, 0, 5, 0}};
    g.loadGraph(graph);

    CHECK(ariel::Algorithms::negativeCycle(g) == "the graph does not contain a negative cycle");
}

TEST_CASE("Test isCyclicUtil with no cycle")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
            {0, 1, 0, 0, 0},
            {1, 0, 3, 0, 0},
            {0, 3, 0, 4, 0},
            {0, 0, 4, 0, 5},
            {0, 0, 0, 5, 0}};
    g.loadGraph(graph);

    std::vector<bool> visited(g.getNumberOfVertices(), false);
    std::vector<bool> recStack(g.getNumberOfVertices(), false);
    std::vector<size_t> parent(g.getNumberOfVertices(), SIZE_MAX);
    std::vector<size_t> cycle;
    CHECK(ariel::Algorithms::isCyclicUtil(0, visited, recStack, parent, g, cycle) == false);
}

TEST_CASE("Test isConnected with disconnected graph") {
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0, 0, 0},
        {1, 0, 0, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isConnected(g) == false);
}

TEST_CASE("Test shortestPath with unreachable vertices") {
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 0},
        {0, 0, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "-1");
}

TEST_CASE("Test isBipartite with a complete bipartite graph") {
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 1, 0},
        {1, 0, 0, 1},
        {1, 0, 0, 1},
        {0, 1, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0, 3}, B={1, 2}");
}

TEST_CASE("Test isBipartite with a non-bipartite graph") {
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 1, 0},
        {1, 0, 1, 1},
        {1, 1, 0, 1},
        {0, 1, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isBipartite(g) == "0");
}
