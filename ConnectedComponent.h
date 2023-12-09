#ifndef CONNECTEDCOMPONENT_H
#define CONNECTEDCOMPONENT_H

#include <vector>

class ConnectedComponent
{
public:
    ConnectedComponent() = default;

    void AddNode(int node);

    std::vector<int> GetNodes() const;

private:
    std::vector<int> m_nodes;
};

#endif // CONNECTEDCOMPONENT_H
