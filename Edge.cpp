#include "Edge.h"

Edge::Edge()
{
    m_weight = FLT_MAX;
}

Edge::Edge(int firstNode, int lastNode, float cost)
{
    m_firstNode = firstNode;
    m_lastNode = lastNode;
    m_weight = cost;
}

int Edge::GetFirstNode() const
{
    return m_firstNode;
}

int Edge::GetLastNode() const
{
    return m_lastNode;
}

float Edge::GetWeight() const
{
    return m_weight;
}

void Edge::SetFirstNode(int node)
{
    m_firstNode = node;
}

void Edge::SetLastNode(int node)
{
    m_lastNode = node;
}

void Edge::SetWeight(float weight)
{
    m_weight = weight;
}
