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
    int n = m_nodes.size();
    if (n == 0)
        return std::vector<Edge>();

    std::vector<float> distance(n, (float)INT_MAX);
    std::vector<int> cameFrom(n, -1);

    distance[start] = 0;

    Graph reversedGraph = !(*this);

    std::vector<float> tempDistance;

    int cnt_cycle = 0;

    do
    {
        if (cnt_cycle > 2 * n)
            return std::vector<Edge>();

        tempDistance = distance;

        for (int y = 0; y < n; ++y)
        {
            std::vector<int> predecessors = reversedGraph.m_adjacencyLists[y];

            for (int x : predecessors)
            {
                Edge edge = GetEdge(x + 1, y + 1);
                float cost = edge.GetWeight();

                if (tempDistance[x] + cost < tempDistance[y])
                {
                    cameFrom[y] = x;
                    distance[y] = tempDistance[x] + cost;
                }
            }
        }

        ++cnt_cycle;
    }
    while (tempDistance != distance);



    // Există drum de la start -> end <=> distanţa finită
    if (distance[end] != FLT_MAX)
    {
        // Arcele care stabilesc drumul între start -> end
        return GetPath(end, cameFrom);
    }

    return std::vector<Edge>();
}

std::vector<Edge> Graph::FloydWarshall(int start, int end)
{
    int n = m_nodes.size();
    if (n == 0)
        return std::vector<Edge>();

    std::vector<std::vector<float>> distance(n);
    std::vector<std::vector<int>> cameFrom(n);

    /*
     * n = 5
     *      [0,0,0,0,0]
     *      [0,0,0,0,0]
     *  d = [0,0,0,0,0]
     *      [0,0,0,0,0]
     *      [0,0,0,0,0]
    */

    for (int i = 0; i < n; ++i)
    {
        distance[i].resize(n);
        cameFrom[i].resize(n);

        for (int j = 0; j < n; ++j)
        {
            distance[i][j] = m_costMatrix[i][j];

            if (i != j && distance[i][j] < FLT_MAX)
            {
                cameFrom[i][j] = i;
            }
            else
            {
                cameFrom[i][j] = -1;
            }
        }
    }

    for (int k = 0; k < n; ++k)
    {
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                if (distance[i][k] + distance[k][j] < distance[i][j])
                {
                    cameFrom[i][j] = cameFrom[k][j];
                    distance[i][j] = distance[i][k] + distance[k][j];
                }
            }
        }
    }

    if (distance[start][end] != FLT_MAX)
        return GetPath(start, end, cameFrom);

    return std::vector<Edge>();
}

std::vector<Edge> Graph::GetPath(int start, int end, const std::vector<std::vector<int> > &cameFrom)
{
    // Se va crea un vector de muchii/arce pe baza nodurilor care alcătuiesc drumul
    std::vector<int> nodes =
        PathHelper::GetPath(start, end, cameFrom);
    if (nodes.empty())
        return std::vector<Edge>();

    std::vector<Edge> edges;
    for (size_t i = 0; i < nodes.size() - 1; ++i)
    {
        int x = nodes[i] + 1;
        int y = nodes[i + 1] + 1;

        Edge edge = GetEdge(x, y);
        edges.push_back(edge);
    }

    return edges;
}
