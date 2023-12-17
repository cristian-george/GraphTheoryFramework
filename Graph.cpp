#include <fstream>

#include "Graph.h"

Graph::Graph()
{
    m_isDirected = false;
    m_isWeighted = false;
}

Node Graph::GetNode(int value)
{
    return m_nodes[value - 1];
}

Edge Graph::GetEdge(int node1, int node2)
{
    for (const Edge& edge : m_edges)
    {
        if (edge.GetFirstNode() == node1 && edge.GetLastNode() == node2)
            return edge;
    }

    return {};
}

std::vector<Node> Graph::GetNodes()
{
    return m_nodes;
}

std::vector<Edge> Graph::GetUniqueEdges()
{
    if (m_isDirected) return m_edges;

    std::vector<Edge> uniqueEdges;

    for (const Edge& edge : m_edges)
    {
        if (std::find_if(uniqueEdges.begin(),
                         uniqueEdges.end(),
                         [&](const Edge& e)
                         { return AreEdgesEqual(e, edge); }) == uniqueEdges.end())
        {
            uniqueEdges.push_back(edge);
        }
    }

    return uniqueEdges;
}

int Graph::GetNodeCloseTo(int x, int y)
{
    for (const Node& node : m_nodes)
    {
        if (abs(node.GetCoordinates().GetX() - x) < 2 * Node::radius + 10 &&
            abs(node.GetCoordinates().GetY() - y) < 2 * Node::radius + 10)
        {
            return node.GetValue();
        }
    }

    return -1;
}

bool Graph::IsDirected() const
{
    return m_isDirected;
}

bool Graph::IsWeighted() const
{
    return m_isWeighted;
}

bool Graph::AddNode(int x, int y)
{
    Point<int> point(x, y);
    if (ExistsNodeCloseTo(point)) return false;

    int value = m_nodes.size() + 1;
    m_nodes.emplace_back(point, value);

    m_adjacencyMatrix.push_back(std::vector<bool>(m_nodes.size()));
    m_adjacencyLists.push_back(std::vector<int>());

    m_costMatrix.push_back(std::vector<float>(m_nodes.size(), FLT_MAX));
    m_costMatrix[m_nodes.size() - 1][m_nodes.size() - 1] = 0;

    return true;
}

bool Graph::AddEdge(int firstNode, int lastNode, float weight)
{
    if (firstNode == lastNode) return false;
    if (ExistsEdge(firstNode, lastNode)) return false;

    m_edges.emplace_back(firstNode, lastNode, weight);

    m_adjacencyMatrix[firstNode - 1][lastNode - 1] = 1;
    m_costMatrix[firstNode - 1][lastNode - 1] = weight;

    m_adjacencyLists[firstNode - 1].push_back(lastNode - 1);

    if (!m_isDirected)
    {
        m_edges.emplace_back(lastNode, firstNode, weight);

        m_adjacencyMatrix[lastNode - 1][firstNode - 1] = 1;
        m_costMatrix[lastNode - 1][firstNode - 1] = weight;

        m_adjacencyLists[lastNode - 1].push_back(firstNode - 1);
    }

    return true;
}

bool Graph::MoveNode(int node, int x, int y)
{
    Point<int> point(x, y);
    m_nodes[node - 1].SetCoordinates(point);

    return true;
}

void Graph::SetDirected(bool directed)
{
    m_isDirected = directed;
}

void Graph::SetWeighted(bool weighted)
{
    m_isWeighted = weighted;
}

void Graph::PrintAdjacencyMatrix()
{
    std::ofstream fout("adjacencyMatrix.out");

    for (size_t row = 0; row < m_adjacencyMatrix.size(); ++row)
    {
        for (size_t column = 0; column < m_adjacencyMatrix.size(); ++column)
        {
            fout << m_adjacencyMatrix[row][column] << " ";
        }

        fout << std::endl;
    }

    fout.close();
}

void Graph::PrintAdjacencyLists()
{
    std::ofstream fout("adjacencyLists.out");

    for (size_t node = 0; node < m_adjacencyLists.size(); ++node)
    {
        fout << "L[" << node + 1 << "]: ";

        // Parcurg lista de adiacenţă a nodului (node + 1)
        for (int neighbour : m_adjacencyLists[node])
        {
            fout << neighbour + 1 << " ";
        }

        fout << std::endl;
    }

    fout.close();
}

void Graph::Clear()
{
    m_nodes.clear();
    m_edges.clear();

    m_adjacencyMatrix.clear();
    m_adjacencyLists.clear();
}


const std::vector<Edge> &Graph::GetEdges()
{
    return m_edges;
}

const Graph::AdjacencyMatrix& Graph::GetAdjacencyMatrix()
{
    return m_adjacencyMatrix;
}

const Graph::AdjacencyLists& Graph::GetAdjacencyLists()
{
    return m_adjacencyLists;
}

bool Graph::ExistsNodeCloseTo(const Point<int>& point)
{
    return GetNodeCloseTo(point.GetX(), point.GetY()) != -1;
}

bool Graph::ExistsEdge(int node1, int node2)
{
    return m_adjacencyMatrix[node1 - 1][node2 - 1];
}

bool Graph::AreEdgesEqual(const Edge &edge1, const Edge &edge2)
{
    if (m_isDirected == false)
        // a1 = [1, 2], a2 = [2, 1]
        // a1 = [3, 4], a2 = [3, 4]
        return (edge1.GetFirstNode() == edge2.GetFirstNode() &&
                edge1.GetLastNode() == edge2.GetLastNode()) ||
               (edge1.GetFirstNode() == edge2.GetLastNode() &&
                edge1.GetLastNode() == edge2.GetFirstNode());

    // a1 = (1, 2), a2 = (1, 2)
    return edge1.GetFirstNode() == edge2.GetFirstNode() &&
           edge1.GetLastNode() == edge2.GetLastNode();
}

void Graph::GenerateAdjacencyMatrix()
{
    m_adjacencyMatrix.clear();
    m_costMatrix.clear();

    size_t numberOfNodes = m_nodes.size();
    if (numberOfNodes)
    {
        // n = 5

        for (size_t row = 0; row < numberOfNodes; ++row)
        {
            m_adjacencyMatrix.push_back(std::vector<bool>(numberOfNodes, 0));

            m_costMatrix.push_back(std::vector<float>(numberOfNodes, FLT_MAX));
            m_costMatrix[row][row] = 0;
        }

        //      [0, 0, 0, 0, 0]
        //      [0, 0, 0, 0, 0]
        // M =  [0, 0, 0, 0, 0]
        //      [0, 0, 0, 0, 0]
        //      [0, 0, 0, 0, 0]

        // m_edges = [(1, 2), (1, 3), (1, 4), (2, 5), (4, 1), (5, 3)]
        for (const Edge& edge : m_edges)
        {
            int firstNode = edge.GetFirstNode();
            int lastNode = edge.GetLastNode();
            float cost = edge.GetWeight();

            m_adjacencyMatrix[firstNode - 1][lastNode - 1] = 1;
            m_costMatrix[firstNode - 1][lastNode - 1] = cost;
        }

        //      [0, 1, 1, 1, 0]
        //      [0, 0, 0, 0, 1]
        // M =  [0, 0, 0, 0, 0]
        //      [1, 0, 0, 0, 0]
        //      [0, 0, 1, 0, 0]
    }
}

void Graph::GenerateAdjacencyList()
{
    m_adjacencyLists.clear();

    size_t numberOfNodes = m_nodes.size();
    if (numberOfNodes)
    {
        // n = 5
        m_adjacencyLists.resize(numberOfNodes);
        //      []
        //      []
        // M =  []
        //      []
        //      []

        // m_edges = [(1, 2), (1, 3), (1, 4), (2, 5), (4, 5), (5, 3)]
        for (const Edge& edge : m_edges)
        {
            int firstNodeValue = edge.GetFirstNode();
            int lastNodeValue = edge.GetLastNode();

            m_adjacencyLists[firstNodeValue - 1].push_back(lastNodeValue - 1);
        }

        // G orientat:
        //      [2, 3, 4]
        //      [5]
        // M =  []
        //      [5]
        //      [3]

        // G neorientat
        //      [2, 3, 4]
        //      [1, 5]
        // M =  [1, 5]
        //      [1, 5]
        //      [2, 3, 4]
    }
}

Graph Graph::operator!()
{
    Graph reversedGraph;
    reversedGraph.SetDirected(m_isDirected);
    reversedGraph.SetWeighted(m_isWeighted);

    for (Node node : m_nodes)
    {
        int x = node.GetCoordinates().GetX();
        int y = node.GetCoordinates().GetY();
        reversedGraph.AddNode(x, y);
    }

    for (size_t node = 0; node < m_nodes.size(); ++node)
    {
        for (int neighbour : m_adjacencyLists[node])
        {
            Edge edge = GetEdge(node, neighbour);
            float cost = edge.GetWeight();
            reversedGraph.AddEdge(neighbour + 1, node + 1, cost);
        }
    }

    return reversedGraph;
}

std::vector<Edge> Graph::GetPathBetween(int start, int end, EPathFinding algorithm)
{
    std::vector<Edge> edges;
    switch (algorithm)
    {
    case EPathFinding::None:
        break;
    case EPathFinding::BFS:
        edges = ShortPathBetween(start, end);
        break;
    }

    return edges;
}
