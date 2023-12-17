#include <queue>

#include "Graph.h"
// #include "DisjointSet.h"

std::vector<Edge> Graph::Prim(int start)
{
    int n = m_nodes.size();
    if (n == 0)
        return std::vector<Edge>();

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
            // primEdges.push_back(Edge(pred + 1, node + 1, costs[node]));
            primEdges.emplace_back(pred + 1, node + 1, costs[node]);
        }

    return primEdges;
}

std::vector<Edge> Graph::Kruskal()
{

}
