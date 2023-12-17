#include <algorithm>

#include "PathHelper.h"

std::vector<int> PathHelper::GetPath(int end, const std::vector<int> &cameFrom)
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

int PathHelper::GetUnvisitedNode(const std::vector<int>& visited)
{
    for (size_t node = 0; node < visited.size(); ++node)
        if (visited[node] == -1)
            return node;

    return INT_MAX;
}

int PathHelper::GetUnanalyzedNeighbour(const std::vector<int>& neighbours,
                                       const std::vector<int>& visited)
{
    for (size_t node = 0; node < neighbours.size(); ++node)
        if (visited[neighbours[node]] != 1)
            return node;

    return INT_MAX;
}

int PathHelper::GetUnvisitedNode(const std::vector<bool>& visited)
{
    for (size_t node = 0; node < visited.size(); ++node)
        if (!visited[node])
            return node;

    return INT_MAX;
}
