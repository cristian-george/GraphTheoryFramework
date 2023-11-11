#ifndef EDGE_H
#define EDGE_H

#include <float.h>

#include "Node.h"

class Edge
{
public:
    Edge();
    Edge(const Node& firstNode, const Node& lastNode, float cost=FLT_MIN);

    const Node& GetFirstNode() const;
    const Node& GetLastNode() const;
    float GetWeight() const;

    void SetFirstNode(const Node& node);
    void SetLastNode(const Node& node);
    void SetWeight(float weight);

private:
    Node m_firstNode;
    Node m_lastNode;
    float m_weight;
};

#endif // EDGE_H
