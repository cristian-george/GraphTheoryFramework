#pragma once

#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <algorithm>

#include "Edge.h"
#include "Node.h"
#include "Enums.h"
#include "ConnectedComponent.h"

class Graph
{
    using AdjacencyMatrix = std::vector<std::vector<bool>>;
    using CostMatrix = std::vector<std::vector<float>>;
    using AdjacencyLists = std::vector<std::vector<int>>;

public:
    Graph();

    Node GetNode(int value);
    Edge GetEdge(int node1, int node2);

    std::vector<Node> GetNodes();
    std::vector<Edge> GetUniqueEdges();

    int GetNodeCloseTo(int x, int y);

    bool IsDirected() const;
    bool IsWeighted() const;

    bool AddNode(int x, int y);
    bool AddEdge(int node1, int node2, float weight = FLT_MAX);

    bool MoveNode(int node, int x, int y);

    void SetDirected(bool directed);
    void SetWeighted(bool weighted);

    void PrintAdjacencyMatrix();
    void PrintAdjacencyLists();

    void Clear();

private:
    const std::vector<Edge>& GetEdges();

    const AdjacencyMatrix& GetAdjacencyMatrix();
    const AdjacencyLists& GetAdjacencyLists();

    bool ExistsNodeCloseTo(const Point<int>& point);
    bool ExistsEdge(int node1, int node2);

    bool AreEdgesEqual(const Edge& edge1, const Edge& edge2);

    void GenerateAdjacencyMatrix();
    void GenerateAdjacencyList();

    // Operatorul ! returnează graful invers
    Graph operator!();

public:
    // Traversal
    std::vector<Edge> ShortPathBetween(int start, int end);
    std::vector<int> TopologicalSort();

    std::vector<ConnectedComponent> ConnectedComponents();
    std::vector<ConnectedComponent> StronglyConnectedComponents();

    // MST
    std::vector<Edge> Prim(int start);
    std::vector<Edge> Kruskal();

    // Minimum cost paths
    std::vector<Edge> Dijkstra(int start, int end);
    std::vector<Edge> BellmanFord(int start, int end);
    std::vector<Edge> FloydWarshall(int start, int end);

    // Finding path
    std::vector<Edge> GetPath(int end,
                              const std::vector<int>& cameFrom);
    std::vector<Edge> GetPath(int start,
                              int end,
                              const std::vector<std::vector<int>>& cameFrom);
    std::vector<Edge> GetPathBetween(int start,
                                     int end,
                                     EPathFinding algorithm);

private:
    // G = (N, A)
    std::vector<Node> m_nodes; // N = mulţimea nodurilor
    std::vector<Edge> m_edges; // A = mulţimea muchiilor/arcelor

    bool m_isDirected; // 1 - orientat, 0 - neorientat
    bool m_isWeighted; // 1 - ponderat, 0 - fără ponderi

    AdjacencyMatrix m_adjacencyMatrix; // matricea de adiacenţă
    CostMatrix m_costMatrix; // matricea de costuri / valoare-adiacenţă
    AdjacencyLists m_adjacencyLists; // listele de adiacenţă
};

#endif // GRAPH_H
