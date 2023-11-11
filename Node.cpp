#include "Node.h"

const int Node::radius = 20;

Node::Node()
{
    m_value = -1;
}

Node::Node(const QPointF &coordinates, int value)
{
    m_coordinates = coordinates;
    m_value = value;
}

QPointF Node::GetCoordinates() const
{
    return m_coordinates;
}

int Node::GetValue() const
{
    return m_value;
}

void Node::SetCoordinates(const QPointF &coordinates)
{
    m_coordinates = coordinates;
}

void Node::SetValue(int value)
{
    m_value = value;
}

bool Node::operator==(const Node &node) const
{
    return m_value == node.m_value;
}

bool Node::operator!=(const Node &node) const
{
    return m_value != node.m_value;
}
