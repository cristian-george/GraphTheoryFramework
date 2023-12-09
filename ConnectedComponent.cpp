#include "ConnectedComponent.h"

void ConnectedComponent::AddNode(int node)
{
    m_nodes.push_back(node);
}

std::vector<int> ConnectedComponent::GetNodes() const
{
    return m_nodes;
}
