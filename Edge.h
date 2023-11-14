#pragma once

#ifndef EDGE_H
#define EDGE_H

#include <float.h>

class Edge
{
public:
    Edge();
    Edge(int firstNode, int lastNode, float cost=FLT_MAX);

    int GetFirstNode() const;
    int GetLastNode() const;
    float GetWeight() const;

    void SetFirstNode(int node);
    void SetLastNode(int node);
    void SetWeight(float weight);

private:
    int m_firstNode;
    int m_lastNode;
    float m_weight;
};

#endif // EDGE_H
