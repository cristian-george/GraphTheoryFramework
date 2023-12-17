#include <queue>

#include "Graph.h"
// #include "DisjointSet.h"

std::vector<Edge> Graph::Prim(int start)
{
    if (m_isDirected)
        throw "Graph is oriented!";

    if (!m_isWeighted)
        throw "Graph is not weighted!";

    int n = m_nodes.size();
    if (n < 2)
        throw "Graph has less than 2 nodes!";

    if (ConnectedComponents().size() != 1)
        throw "Graph is not connected!";

    // A' = în acest vector reţinem muchiile ce formează arborele parţial
    std::vector<Edge> primEdges;

    std::vector<bool> visited(n, false);

    std::vector<float> costs(n, FLT_MAX);
    std::vector<int> cameFrom(n, -1);

    costs[start] = 0;

    // În pq adaug perechi (cost, nod)
    // pq utilizează primele valori ale perechilor pentru actualizarea min-heap-ului
    std::priority_queue<std::pair<float, int>,
                        std::vector<std::pair<float, int>>,
                        std::greater<std::pair<float, int>>> pq;

    pq.push(std::make_pair(costs[start], start));

    while (!pq.empty())
    {
        int currentNode = pq.top().second;
        pq.pop();

        if (visited[currentNode])
            continue;

        for (int neighbour : m_adjacencyLists[currentNode])
        {
            Edge edge = GetEdge(currentNode + 1, neighbour + 1);
            float cost = edge.GetWeight();

            if (!visited[neighbour] && cost < costs[neighbour])
            {
                cameFrom[neighbour] = currentNode;
                costs[neighbour] = cost;
                pq.push(std::make_pair(cost, neighbour));
            }
        }

        visited[currentNode] = true;
    }

    // cameFrom[start] = -1
    for (int node = 0; node < n; ++node)
        if (node != start)
        {
            int pred = cameFrom[node];
            primEdges.emplace_back(pred + 1, node + 1, costs[node]);
        }

    return primEdges;
}

std::vector<Edge> Graph::Kruskal()
{

}
