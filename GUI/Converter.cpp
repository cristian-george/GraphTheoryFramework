#include <QPoint>

#include "Point.h"

Point<int> ToPoint(const QPoint &point)
{
    return Point<int>(point.x(), point.y());
}

QPoint ToQPoint(const Point<int> &point)
{
    return QPoint(point.GetX(), point.GetY());
}
