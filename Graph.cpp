#include "Graph.h"

Graph::Graph()
{
    m_isDirected = false;
    m_isWeighted = false;
}

void Graph::AddNode(const Node& node)
{
    m_nodes.push_back(node);

    size_t numberOfNodes = m_nodes.size();
    m_nodes[numberOfNodes - 1].SetValue(numberOfNodes);
}

void Graph::AddEdge(const Edge& edge)
{
    m_edges.push_back(edge);
}

std::vector<Node>& Graph::GetNodes()
{
    return m_nodes;
}

std::vector<Edge>& Graph::GetEdges()
{
    return m_edges;
}

void Graph::SetDirected(bool directed)
{
    m_isDirected = directed;
}

void Graph::SetWeighted(bool weighted)
{
    m_isWeighted = weighted;
}

bool Graph::IsDirected() const
{
    return m_isDirected;
}

bool Graph::IsWeighted() const
{
    return m_isWeighted;
}

void Graph::GenerateAdjacencyMatrix()
{
    m_adjacencyMatrix.clear();

    size_t numberOfNodes = m_nodes.size();
    if (numberOfNodes)
    {
        m_adjacencyMatrix.resize(numberOfNodes);

        for (size_t row = 0; row < numberOfNodes; ++row)
            m_adjacencyMatrix[row].resize(numberOfNodes);

        for (const Edge& edge : m_edges)
        {
            int firstNodeValue = edge.GetFirstNode().GetValue();
            int lastNodeValue = edge.GetLastNode().GetValue();

            m_adjacencyMatrix[firstNodeValue - 1][lastNodeValue - 1] = 1;
        }
    }
}

void Graph::GenerateAdjacencyList()
{
    m_adjacencyList.clear();

    size_t numberOfNodes = m_nodes.size();
    if (numberOfNodes)
    {
        m_adjacencyList.resize(numberOfNodes);

        for (const Edge& edge : m_edges)
        {
            int firstNodeValue = edge.GetFirstNode().GetValue();
            int lastNodeValue = edge.GetLastNode().GetValue();

            m_adjacencyList[firstNodeValue - 1].push_back(lastNodeValue - 1);
        }
    }
}

std::vector<std::vector<bool>>& Graph::GetAdjacencyMatrix()
{
    return m_adjacencyMatrix;
}

std::vector<std::vector<int>>& Graph::GetAdjacencyList()
{
    return m_adjacencyList;
}

bool Graph::ExistsNodeCloseTo(const Node& otherNode)
{
    for (const Node& node : m_nodes)
    {
        if (abs(node.GetCoordinates().x() - otherNode.GetCoordinates().x()) < 2 * Node::radius + 10 &&
            abs(node.GetCoordinates().y() - otherNode.GetCoordinates().y()) < 2 * Node::radius + 10)
            return true;
    }

    return false;
}

bool Graph::ExistsEdge(const Edge& otherEdge)
{
    int firstNodeValue = otherEdge.GetFirstNode().GetValue();
    int lastNodeValue = otherEdge.GetLastNode().GetValue();

    if (m_adjacencyMatrix[firstNodeValue - 1][lastNodeValue - 1] == 1)
        return true;

    return false;
}

bool Graph::AreEdgesEqual(const Edge& edge1, const Edge& edge2)
{
    if (m_isDirected == false)
        return (edge1.GetFirstNode() == edge2.GetFirstNode() && edge1.GetLastNode() == edge2.GetLastNode()) ||
               (edge1.GetFirstNode() == edge2.GetLastNode() && edge1.GetLastNode() == edge2.GetFirstNode());

    return edge1.GetFirstNode() == edge2.GetFirstNode() && edge1.GetLastNode() == edge2.GetLastNode();
}

std::vector<Edge> Graph::GetUniqueEdges()
{
    if (m_isDirected == false)
    {
        std::vector<Edge> uniqueEdges;

        for (const Edge& edge : m_edges)
        {
            if (std::find_if(uniqueEdges.begin(),
                             uniqueEdges.end(),
                             [&](const Edge& e) { return AreEdgesEqual(e, edge); }) == uniqueEdges.end())
            {
                uniqueEdges.push_back(edge);
            }
        }

        return uniqueEdges;
    }

    return m_edges;
}

void Graph::Clear()
{
    m_nodes.clear();
    m_edges.clear();
}
