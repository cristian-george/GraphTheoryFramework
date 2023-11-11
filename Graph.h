#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <algorithm>

#include "Edge.h"
#include "Node.h"

class Graph
{
public:
    Graph();

    void AddNode(const Node& node);
    void AddEdge(const Edge& edge);

    std::vector<Node>& GetNodes();
    std::vector<Edge>& GetEdges();

    void SetDirected(bool directed);
    void SetWeighted(bool weighted);

    bool IsDirected() const;
    bool IsWeighted() const;

    void GenerateAdjacencyMatrix();
    void GenerateAdjacencyList();

    void PrintAdjacencyMatrix();
    void PrintAdjacencyLists();

    std::vector<std::vector<bool>>& GetAdjacencyMatrix();
    std::vector<std::vector<int>>& GetAdjacencyList();

    bool ExistsNodeCloseTo(const Node& otherNode);
    bool ExistsEdge(const Edge& otherEdge);

    bool AreEdgesEqual(const Edge& edge1, const Edge& edge2);
    std::vector<Edge> GetUniqueEdges();

    void Clear();

public:
    std::vector<int> BFS(int start, int end);

private:
    // G = (N, A)
    std::vector<Node> m_nodes; // N = mulţimea nodurilor
    std::vector<Edge> m_edges; // A = mulţimea muchiilor/arcelor

    bool m_isDirected; // 1 - orientat, 0 - neorientat
    bool m_isWeighted; // 1 - ponderat, 0 - fără ponderi

    std::vector<std::vector<bool>> m_adjacencyMatrix;
    std::vector<std::vector<int>> m_adjacencyList;
};

#endif // GRAPH_H
