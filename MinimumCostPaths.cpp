#include <queue>

#include "Graph.h"
#include "PathHelper.h"

std::vector<Edge> Graph::Dijkstra(int start, int end)
{
    int n = m_nodes.size();
    if (n == 0)
        return std::vector<Edge>();

    std::vector<float> distance(n, FLT_MAX);
    std::vector<int> cameFrom(n, -1);

    // În pq adaug perechi (distanţă, nod)
    // pq utilizează primele valori ale perechilor pentru actualizarea min-heap-ului
    std::priority_queue<std::pair<float, int>,
                        std::vector<std::pair<float, int>>,
                        std::greater<std::pair<float, int>>> pq;

    distance[start] = 0;

    pq.push(std::make_pair(distance[start], start));

    while (!pq.empty())
    {
        int x = pq.top().second;
        pq.pop();

        if (x == end)
            break;

        for (int y : m_adjacencyLists[x])
        {
            Edge edge = GetEdge(x + 1, y + 1);
            float cost = edge.GetWeight();

            if (distance[x] + cost < distance[y])
            {
                cameFrom[y] = x;
                distance[y] = distance[x] + cost;
                pq.push(std::make_pair(distance[y], y));
            }
        }
    }

    // Există drum de la start -> end <=> distanţa finită
    if (distance[end] != FLT_MAX)
    {
        // Arcele care stabilesc drumul între start -> end
        return GetPath(end, cameFrom);
    }

    return std::vector<Edge>();
}

std::vector<Edge> Graph::BellmanFord(int start, int end)
{

}

std::vector<Edge> Graph::FloydWarshall(int start, int end)
{

}
