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

    std::vector<std::vector<bool>>& GetAdjacencyMatrix();
    std::vector<std::vector<int>>& GetAdjacencyList();

    bool ExistsNodeCloseTo(const Node& otherNode);
    bool ExistsEdge(const Edge& otherEdge);

    bool AreEdgesEqual(const Edge& edge1, const Edge& edge2);
    std::vector<Edge> GetUniqueEdges();

    void Clear();

private:
    std::vector<Node> m_nodes;
    std::vector<Edge> m_edges;

    bool m_isDirected;
    bool m_isWeighted;

    std::vector<std::vector<bool>> m_adjacencyMatrix;
    std::vector<std::vector<int>> m_adjacencyList;
};

#endif // GRAPH_H
