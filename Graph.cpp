#include <fstream>
#include <queue>
#include <stack>
#include <numeric>

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

    return true;
}

bool Graph::AddEdge(int node1, int node2, float weight)
{
    if (node1 == node2) return false;
    if (ExistsEdge(node1, node2)) return false;

    m_edges.emplace_back(node1, node2, weight);

    m_adjacencyMatrix[node1 - 1][node2 - 1] = 1;
    m_adjacencyLists[node1 - 1].push_back(node2 - 1);

    if (!m_isDirected)
    {
        m_edges.emplace_back(node2, node1, weight);

        m_adjacencyMatrix[node2 - 1][node1 - 1] = 1;
        m_adjacencyLists[node2 - 1].push_back(node1 - 1);
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
            int firstNodeValue = edge.GetFirstNode();
            int lastNodeValue = edge.GetLastNode();

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


std::vector<int> Graph::ShortPathBetween(int start, int end)
{
    int numberOfNodes = m_nodes.size();

    std::vector<int> cameFrom(numberOfNodes);
    std::vector<int> distance(numberOfNodes);

    for (int index = 0; index < numberOfNodes; ++index)
    {
        distance[index] = INT_MAX;
        cameFrom[index] = -1;
    }

    distance[start] = 0;

    std::queue<int> Q;
    Q.push(start + 1);

    // Parcurgerea în lăţime
    while (!Q.empty())
    {
        int currentNode = Q.front() - 1;
        Q.pop();

        std::vector<int> neighbours = m_adjacencyLists[currentNode];
        for (int neighbour : neighbours)
        {
            if (distance[neighbour] == INT_MAX)
            {
                Q.push(neighbour + 1);
                distance[neighbour] = distance[currentNode] + 1;
                cameFrom[neighbour] = currentNode;
            }
        }
    }

    if (distance[end] != INT_MAX)
    {
        std::vector<int> path = GetBfsPath(end, cameFrom);
        if (!path.empty()) // Dacă există drum de la "start" la "end" atunci se va afişa în fişier
        {
            std::ofstream fout("bfs_path.out");

            fout << "Path from node " << start + 1 << " to node " << end + 1 << ": ";
            for (int node : path)
                fout << node + 1 << " ";

            return path;
        }
    }

    return std::vector<int>();
}

std::vector<int> Graph::TopologicalSort()
{
    // În acest vector vom reţine nodurile în ordinea sortării topologice
    std::vector<int> topologicalSort;

    int numberOfNodes = m_nodes.size();
    std::vector<int> flag(numberOfNodes);
    /* -1 - nevizitat
    /   0 - vizitat, dar neanalizat
    /   1 - vizitat şi analizat
    */

    for (int index = 0; index < numberOfNodes; ++index)
    {
        flag[index] = -1;
    }

    int start = GetUnvisitedNode(flag);

    std::stack<int> S;

    if (start != INT_MAX)
    {
        flag[start] = 0;
        S.push(start);
    }

    // Parcurgerea totală în adâncime
    while (start != INT_MAX)
    {
        while (!S.empty())
        {
            int currentNode = S.top();

            std::vector<int> neighbours = m_adjacencyLists[currentNode];
            for (int neighbour : neighbours)
            {
                if (flag[neighbour] == -1) // Am găsit un vecin nevizitat
                {
                    flag[neighbour] = 0; // Vizitat, dar neanalizat
                    S.push(neighbour);
                    break;
                }
                else if (flag[neighbour] == 0) // Am găsit un ciclu
                {
                    /* Ciclul este determinat prin eliminarea nodurilor inserate în stivă.
                     * Ne oprim când valoarea din capul stivei este egală cu "neighbour"
                    */

                    std::vector<int> cycle;
                    cycle.push_back(neighbour);

                    while (!S.empty())
                    {
                        int value = S.top();
                        cycle.push_back(value);
                        if (value == neighbour)
                            break;

                        S.pop();
                    }

                    std::reverse(cycle.begin(), cycle.end());

                    std::ofstream fout("dfs_path.out");

                    fout << "Found a cycle: ";
                    for (int node : cycle)
                        fout << node + 1 << " ";

                    topologicalSort.clear();
                    return std::vector<int>();
                }
            }

            int unanalyzedNeighbour = GetUnanalyzedNeighbour(neighbours, flag);
            if (neighbours.empty() ||
                (!neighbours.empty() && unanalyzedNeighbour == INT_MAX))
            {
                /* Dacă nodul curent nu are vecini SAU
                /  Dacă nodul curent are toţi vecinii vizitaţi şi analizaţi
                /  atunci nodul curent va fi setat ca vizitat şi analizat
                */
                flag[currentNode] = 1;
                topologicalSort.push_back(currentNode);
                S.pop();
            }
        }

        start = GetUnvisitedNode(flag);
        if (start != INT_MAX)
        {
            flag[start] = 0;
            S.push(start);
        }
    }

    std::reverse(topologicalSort.begin(), topologicalSort.end());

    std::ofstream fout("dfs_path.out");

    fout << "Topological sort: ";
    for (int node : topologicalSort)
    {
        fout << node + 1 << " ";
    }

    return topologicalSort;
}

std::vector<ConnectedComponent> Graph::ConnectedComponents()
{
    std::vector<int> flag(m_nodes.size(), -1);
    /* -1 - nevizitat
    /   0 - vizitat, dar neanalizat
    /   1 - vizitat şi analizat
    */

    int start = GetUnvisitedNode(flag);
    if (start == INT_MAX)
        return std::vector<ConnectedComponent>();

    std::vector<ConnectedComponent> CC;

    std::queue<int> Q;
    Q.push(start);
    flag[start] = 0;

    // Parcurgerea totală în lăţime
    while (start != INT_MAX)
    {
        ConnectedComponent cc;

        // Parcurgerea în lăţime
        while (!Q.empty())
        {
            int currentNode = Q.front();

            cc.AddNode(currentNode + 1);

            std::vector<int> neighbours = m_adjacencyLists[currentNode];
            for (int neighbour : neighbours)
            {
                if (flag[neighbour] == -1)
                {
                    Q.push(neighbour);
                    flag[neighbour] = 0;
                }
            }

            Q.pop();
            flag[currentNode] = 1;
        }

        CC.push_back(cc);

        start = GetUnvisitedNode(flag);
        if (start != INT_MAX)
        {
            Q.push(start);
            flag[start] = 0;
        }
    }

    return CC;
}

std::vector<ConnectedComponent> Graph::StronglyConnectedComponents()
{
    std::vector<bool> flag(m_nodes.size(), false);
    /* false (0) - vizitat şi neanalizat
     * true (1) - vizitat şi analizat
    */

    int start = GetUnvisitedNode(flag);
    if (start == INT_MAX)
        return std::vector<ConnectedComponent>();

    std::vector<int> t1(m_nodes.size(), -1);
    std::vector<int> t2(m_nodes.size(), -1);
    int t = 1;

    std::stack<int> S;
    S.push(start);

    // Parcurgerea totală în adâncime
    // Se determină tablourile t1 şi t2 (care au fiecare n elemente)
    // În t1(x) se reţine momentul în care x devine nod vizitat şi neanalizat,
    // iar în t2(x) momentul când x devine vizitat şi analizat.
    while (start != INT_MAX)
    {
        // Parcurgerea în adâncime
        while (!S.empty())
        {
            int currentNode = S.top();

            if (!flag[currentNode])
            {
                flag[currentNode] = true;
                t1[currentNode] = t++;
            }
            else
            {
                S.pop();
                t2[currentNode] = t++;
            }

            std::vector<int> neighbours = m_adjacencyLists[currentNode];
            for (int neighbour : neighbours)
            {
                if (!flag[neighbour])
                    S.push(neighbour);
            }
        }

        start = GetUnvisitedNode(flag);
        if (start != INT_MAX)
            S.push(start);
    }

    Graph reversedGraph = !(*this);

    // Sortăm nodurile în funcţie de timpii t2
    std::vector<int> nodes(m_nodes.size());
    std::iota(nodes.begin(), nodes.end(), 0); // 0, 1, 2, ...., |N|-1

    std::sort(nodes.begin(), nodes.end(), [&t2](int i, int j)
              {
                  return t2[i] > t2[j];
              });

    std::fill(flag.begin(), flag.end(), false);

    std::vector<ConnectedComponent> CTC;

    start = INT_MAX;
    for (int node : nodes)
        if (!flag[node])
        {
            start = node;
            break;
        }

    S.push(start);

    // Parcurgerea totală în adâncime
    // Determinarea componentelor tare conexe
    while (start != INT_MAX)
    {
        ConnectedComponent ctc;

        // Parcurgerea în adâncime
        while (!S.empty())
        {
            int currentNode = S.top();

            if (!flag[currentNode])
            {
                flag[currentNode] = true;
                ctc.AddNode(currentNode + 1);
            }
            else
            {
                S.pop();
            }

            std::vector<int> neighbours = reversedGraph.m_adjacencyLists[currentNode];
            for (int neighbour : neighbours)
            {
                if (!flag[neighbour])
                    S.push(neighbour);
            }
        }

        CTC.push_back(ctc);

        start = INT_MAX;
        for (int node : nodes)
            if (!flag[node])
            {
                start = node;
                break;
            }

        if (start != INT_MAX)
            S.push(start);
    }

    return CTC;
}


std::vector<int> Graph::GetBfsPath(int end, const std::vector<int> &cameFrom)
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

    return path;
}

int Graph::GetUnvisitedNode(const std::vector<int>& visited)
{
    for (size_t node = 0; node < visited.size(); ++node)
        if (visited[node] == -1)
            return node;

    return INT_MAX;
}

int Graph::GetUnanalyzedNeighbour(const std::vector<int>& neighbours,
                                  const std::vector<int>& visited)
{
    for (size_t node = 0; node < neighbours.size(); ++node)
        if (visited[neighbours[node]] != 1)
            return node;

    return INT_MAX;
}

int Graph::GetUnvisitedNode(const std::vector<bool>& visited)
{
    for (size_t node = 0; node < visited.size(); ++node)
        if (!visited[node])
            return node;

    return INT_MAX;
}
