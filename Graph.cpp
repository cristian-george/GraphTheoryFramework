#include <fstream>
#include <queue>

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
        // n = 5
        m_adjacencyMatrix.resize(numberOfNodes);
        //      []
        //      []
        // M =  []
        //      []
        //      []

        for (size_t row = 0; row < numberOfNodes; ++row)
            m_adjacencyMatrix[row].resize(numberOfNodes);

        //      [0, 0, 0, 0, 0]
        //      [0, 0, 0, 0, 0]
        // M =  [0, 0, 0, 0, 0]
        //      [0, 0, 0, 0, 0]
        //      [0, 0, 0, 0, 0]

        // m_edges = [(1, 2), (1, 3), (1, 4), (2, 5), (4, 1), (5, 3)]
        for (const Edge& edge : m_edges)
        {
            int firstNodeValue = edge.GetFirstNode().GetValue();
            int lastNodeValue = edge.GetLastNode().GetValue();

            m_adjacencyMatrix[firstNodeValue - 1][lastNodeValue - 1] = 1;
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
    m_adjacencyList.clear();

    size_t numberOfNodes = m_nodes.size();
    if (numberOfNodes)
    {
        // n = 5
        m_adjacencyList.resize(numberOfNodes);
        //      []
        //      []
        // M =  []
        //      []
        //      []

        // m_edges = [(1, 2), (1, 3), (1, 4), (2, 5), (4, 1), (5, 3)]
        for (const Edge& edge : m_edges)
        {
            int firstNodeValue = edge.GetFirstNode().GetValue();
            int lastNodeValue = edge.GetLastNode().GetValue();

            m_adjacencyList[firstNodeValue - 1].push_back(lastNodeValue - 1);
        }

        //      [2, 3, 4]
        //      [5]
        // M =  []
        //      [1]
        //      [3]
    }
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

    for (size_t node = 0; node < m_adjacencyList.size(); ++node)
    {
        fout << "L[" << node + 1 << "]: ";

        // Parcurg lista de adiacenţă a nodului (node + 1)
        for (int neighbour : m_adjacencyList[node])
        {
            fout << neighbour + 1 << " ";
        }

        fout << std::endl;
    }

    fout.close();
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
        // a1 = [1, 2], a2 = [2, 1]
        // a1 = [1, 2], a2 = [1, 2]
        return (edge1.GetFirstNode() == edge2.GetFirstNode() && edge1.GetLastNode() == edge2.GetLastNode()) ||
               (edge1.GetFirstNode() == edge2.GetLastNode() && edge1.GetLastNode() == edge2.GetFirstNode());

    // a1 = (1, 2), a2 = (1, 2)
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

std::vector<int> Graph::BFS(int start, int end)
{
    int numberOfNodes = m_nodes.size();

    std::vector<int> cameFrom(numberOfNodes + 1);
    std::vector<int> distance(numberOfNodes + 1);

    for (int index = 1; index <= numberOfNodes; ++index)
    {
        distance[index] = INT_MAX;
        cameFrom[index] = -1;
    }

    distance[start] = 0;

    std::queue<int> Q;
    Q.push(start);

    while (!Q.empty())
    {
        int currentNode = Q.front();
        Q.pop();

        std::vector<int> neighbours = m_adjacencyList[currentNode - 1];
        for (int neighbour : neighbours)
        {
            if (distance[neighbour + 1] == INT_MAX)
            {
                Q.push(neighbour + 1);
                distance[neighbour + 1] = distance[currentNode] + 1;
                cameFrom[neighbour + 1] = currentNode;
            }
        }
    }

    if (distance[end] != INT_MAX)
    {
        std::vector<int> path;

        int currentNode = end;
        int pred = cameFrom[currentNode];

        if (pred == -1)
            return std::vector<int>();

        path.push_back(currentNode);

        while (pred != -1)
        {
            path.push_back(pred);
            pred = cameFrom[pred];
        }

        std::reverse(path.begin(), path.end());

        std::ofstream fout("bfs_path.out");

        fout << "Path from node " << start << " to node " << end << ": ";
        for (int node : path)
            fout << node << " ";

        return path;
    }

    return std::vector<int>();
}
