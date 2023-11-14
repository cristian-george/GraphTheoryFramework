#include <QPainter>
#include <QPainterPath>

#include "Converter.cpp"
#include "MainWindow.h"

void MainWindow::drawNode(const Node &node, Qt::GlobalColor color)
{
    QPainter painter(this);

    int x = node.GetCoordinates().GetX();
    int y = node.GetCoordinates().GetY();

    QRect rect(x - Node::radius,
               y - Node::radius,
               2 * Node::radius,
               2 * Node::radius);

    painter.setPen(QPen(Qt::black));
    painter.setBrush(QBrush(color));
    painter.drawEllipse(rect);

    painter.setFont(QFont("Times", 12, QFont::Bold));
    painter.drawText(rect, Qt::AlignCenter, QString::number(node.GetValue()));
}


void MainWindow::drawEdge(const Edge &edge, Qt::GlobalColor color)
{
    QPoint p1 = ToQPoint(graph.GetNode(edge.GetFirstNode()).GetCoordinates());
    QPoint p2 = ToQPoint(graph.GetNode(edge.GetLastNode()).GetCoordinates());

    QPen pen(color, 2);

    QPainter painter(this);
    painter.setPen(pen);

    QLineF straightLine(p1, p2);
    straightLine.setLength(straightLine.length() - Node::radius);

    qreal mX = (p1.x() + straightLine.p2().x()) / 2;
    qreal mY = (p1.y() + straightLine.p2().y()) / 2;

    qreal dX = p2.x() - p1.x();
    qreal dY = p2.y() - p1.y();

    qreal distance = sqrt(pow(dX, 2) + pow(dY, 2));
    qreal curvature = 10;

    qreal cX = -distance / curvature * (dY / distance) + mX;
    qreal cY = distance / curvature * (dX / distance) + mY;

    if (graph.IsWeighted())
    {
        painter.setFont(QFont("Times", 12, QFont::Bold));
        painter.drawText(
            QPointF((mX + cX) / 2, (mY + cY) / 2),
            QString::number(edge.GetWeight()));
    }

    if (graph.IsDirected())
    {
        QPointF controlPoint(cX, cY);
        QLineF ghostLine(controlPoint, p2);

        ghostLine.setLength(ghostLine.length() - Node::radius);

        double angle = ::acos(ghostLine.dx() / ghostLine.length());

        if (ghostLine.dy() >= 0)
            angle = (M_PI * 2) - angle;

        qreal arrowSize = 10;
        QPointF arrowP1 = ghostLine.p2() -
                          QPointF(sin(angle + M_PI / 3) * arrowSize,
                                  cos(angle + M_PI / 3) * arrowSize);
        QPointF arrowP2 = ghostLine.p2() -
                          QPointF(sin(angle + M_PI - M_PI / 3) * arrowSize,
                                  cos(angle + M_PI - M_PI / 3) * arrowSize);

        QPainterPath curvedLine;
        curvedLine.moveTo(p1);
        curvedLine.quadTo(controlPoint, ghostLine.p2());

        painter.strokePath(curvedLine, pen);
        painter.drawLine(ghostLine.p2(), arrowP1);
        painter.drawLine(ghostLine.p2(), arrowP2);
    }
    else
    {
        painter.drawLine(straightLine);
    }
}
