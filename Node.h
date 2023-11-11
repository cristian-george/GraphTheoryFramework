#ifndef NODE_H
#define NODE_H

#include <QPoint>

class Node
{
public:
    Node();
    Node(const QPointF& coordinates, int value = -1);

    QPointF GetCoordinates() const;
    int GetValue() const;

    void SetCoordinates(const QPointF& coordinates);
    void SetValue(int value);

    bool operator==(const Node& node) const;
    bool operator!=(const Node& node) const;

public:
    static const int radius;

private:
    QPointF m_coordinates;
    int m_value;
};

#endif // NODE_H
