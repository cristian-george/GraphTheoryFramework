#include "Edge.h"

Edge::Edge()
{
    m_weight = FLT_MIN;
}

Edge::Edge(const Node& firstNode, const Node& lastNode, float cost)
{
    m_firstNode = firstNode;
    m_lastNode = lastNode;
    m_weight = cost;
}

const Node& Edge::GetFirstNode() const
{
    return m_firstNode;
}

const Node& Edge::GetLastNode() const
{
    return m_lastNode;
}

float Edge::GetWeight() const
{
    return m_weight;
}

void Edge::SetFirstNode(const Node &node)
{
    m_firstNode = node;
}

void Edge::SetLastNode(const Node &node)
{
    m_lastNode = node;
}

void Edge::SetWeight(float weight)
{
    m_weight = weight;
}
