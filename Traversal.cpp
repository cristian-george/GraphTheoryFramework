#include <queue>
#include <stack>
#include <fstream>
#include <numeric>

#include "Graph.h"
#include "PathHelper.h"

std::vector<Edge> Graph::ShortPathBetween(int start, int end)
{
    int n = m_nodes.size();
    if (n == 0)
        throw "Graph is empty!";

    std::vector<int> cameFrom(n);
    std::vector<int> distance(n);

    for (int index = 0; index < n; ++index)
    {
        distance[index] = INT_MAX;
        cameFrom[index] = -1;
    }

    distance[start] = 0;

    std::queue<int> Q;
    Q.push(start);

    // Parcurgerea în lăţime
    while (!Q.empty())
    {
        int currentNode = Q.front();
        Q.pop();

        std::vector<int> neighbours = m_adjacencyLists[currentNode];
        for (int neighbour : neighbours)
        {
            if (distance[neighbour] == INT_MAX)
            {
                Q.push(neighbour);
                distance[neighbour] = distance[currentNode] + 1;
                cameFrom[neighbour] = currentNode;
            }
        }
    }

    if (distance[end] != INT_MAX)
        return GetPath(end, cameFrom);

    throw "There is no path available!";
}

std::vector<int> Graph::TopologicalSort()
{
    int n = m_nodes.size();
    if (n == 0)
        return std::vector<int>();

    // În acest vector vom reţine nodurile în ordinea sortării topologice
    std::vector<int> topologicalSort;

    std::vector<int> flag(n);
    /* -1 - nevizitat
    /   0 - vizitat, dar neanalizat
    /   1 - vizitat şi analizat
    */

    for (int index = 0; index < n; ++index)
    {
        flag[index] = -1;
    }

    int start = PathHelper::GetUnvisitedNode(flag);

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

                    std::ofstream fout("topo_sort.out");

                    fout << "Found a cycle: ";
                    for (int node : cycle)
                        fout << node + 1 << " ";

                    topologicalSort.clear();
                    return std::vector<int>();
                }
            }

            int unanalyzedNeighbour = PathHelper::GetUnanalyzedNeighbour(neighbours, flag);
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

        start = PathHelper::GetUnvisitedNode(flag);
        if (start != INT_MAX)
        {
            flag[start] = 0;
            S.push(start);
        }
    }

    std::reverse(topologicalSort.begin(), topologicalSort.end());

    std::ofstream fout("topo_sort.out");

    fout << "Topological sort: ";
    for (int node : topologicalSort)
    {
        fout << node + 1 << " ";
    }

    return topologicalSort;
}

std::vector<ConnectedComponent> Graph::ConnectedComponents()
{
    std::vector<bool> flag(m_nodes.size(), false);
    /* false (0) - vizitat şi neanalizat
     * true (1) - vizitat şi analizat
    */

    int start = PathHelper::GetUnvisitedNode(flag);
    if (start == INT_MAX)
        return std::vector<ConnectedComponent>();

    std::vector<ConnectedComponent> CC;

    std::queue<int> Q;
    Q.push(start);

    // Parcurgerea totală în lăţime
    while (start != INT_MAX)
    {
        ConnectedComponent cc;

        // Parcurgerea în lăţime
        while (!Q.empty())
        {
            int currentNode = Q.front();

            if (!flag[currentNode])
            {
                flag[currentNode] = true;
                cc.AddNode(currentNode + 1);
            }
            else
            {
                Q.pop();
            }

            std::vector<int> neighbours = m_adjacencyLists[currentNode];
            for (int neighbour : neighbours)
            {
                if (!flag[neighbour])
                    Q.push(neighbour);
            }
        }

        CC.push_back(cc);

        start = PathHelper::GetUnvisitedNode(flag);
        if (start != INT_MAX)
            Q.push(start);
    }

    return CC;
}

std::vector<ConnectedComponent> Graph::StronglyConnectedComponents()
{
    std::vector<bool> flag(m_nodes.size(), false);
    /* false (0) - vizitat şi neanalizat
     * true (1) - vizitat şi analizat
    */

    int start = PathHelper::GetUnvisitedNode(flag);
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

        start = PathHelper::GetUnvisitedNode(flag);
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

std::vector<Edge> Graph::GetPath(int end, const std::vector<int> &cameFrom)
{
    // Se va crea un vector de muchii/arce pe baza nodurilor care alcătuiesc drumul
    std::vector<int> nodes = PathHelper::GetPath(end, cameFrom);

    if (nodes.empty())
        return std::vector<Edge>();

    std::vector<Edge> edges;
    for (size_t i = 0; i < nodes.size() - 1; ++i)
    {
        int x = nodes[i] + 1;
        int y = nodes[i + 1] + 1;

        auto edge = GetEdge(x, y);
        edges.push_back(edge);
    }

    return edges;
}
