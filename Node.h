#pragma once

#ifndef NODE_H
#define NODE_H

#include "Point.h"

class Node
{
public:
    Node();
    Node(const Point<int>& coordinates, int value = -1);

    Point<int> GetCoordinates() const;
    int GetValue() const;

    void SetCoordinates(const Point<int>& coordinates);
    void SetValue(int value);

    bool operator==(const Node& node) const;
    bool operator!=(const Node& node) const;

public:
    static const int radius;

private:
    Point<int> m_coordinates;
    int m_value;
};

#endif // NODE_H
