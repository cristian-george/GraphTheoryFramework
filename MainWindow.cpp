#include <QPainter>
#include <QPainterPath>
#include <QDebug>
#include <QMessageBox>

#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->inputWeightLabel->setHidden(true);
    ui->inputWeightTextEdit->setHidden(true);

    ui->firstNodeLabel->setText("First node: " + QString::number(firstNode.GetValue()));
    ui->lastNodeLabel->setText("Last node: " + QString::number(lastNode.GetValue()));

    isDrawingNode = false;
    isDrawingEdge = false;

    isBfsRunning = false;

    connectSlots();
}

MainWindow::~MainWindow()
{
    delete timer;
    delete ui;
}

void MainWindow::connectSlots()
{
    connect(
        ui->undirectedRadioButton, &QRadioButton::clicked,
        this, &MainWindow::undirectedRadioButtonClicked);

    connect(
        ui->directedRadioButton, &QRadioButton::clicked,
        this, &MainWindow::directedRadioButtonClicked);

    connect(
        ui->unweightedRadioButton, &QRadioButton::clicked,
        this, &MainWindow::unweightedRadioButtonClicked);

    connect(
        ui->weightedRadioButton, &QRadioButton::clicked,
        this, &MainWindow::weightedRadioButtonClicked);
}

void MainWindow::addNode(QPoint point)
{
    Node node(point);

    if (graph.ExistsNodeCloseTo(node) == false)
    {
        isDrawingNode = true;

        graph.AddNode(node);
        update();
    }
}


void MainWindow::moveNode(QPoint point)
{
    firstNode.SetCoordinates(point);
    graph.GetNodes()[firstNode.GetValue() - 1] = firstNode;

    for(size_t index = 0; index < graph.GetEdges().size(); ++index)
    {
        if (graph.GetEdges()[index].GetFirstNode().GetValue() == firstNode.GetValue())
            graph.GetEdges()[index].SetFirstNode(firstNode);
        else
            if (graph.GetEdges()[index].GetLastNode().GetValue() == firstNode.GetValue())
                graph.GetEdges()[index].SetLastNode(firstNode);
    }

    update();
    firstNode = Node();
}

void MainWindow::addEdge()
{
    if (graph.ExistsEdge(Edge(firstNode, lastNode)) == false && firstNode != lastNode)
    {
        float weight = FLT_MIN;
        if (graph.IsWeighted())
            weight = ui->inputWeightTextEdit->toPlainText().toFloat();

        if (graph.IsDirected() == true)
            graph.AddEdge(Edge(firstNode, lastNode, weight));
        else
        {
            graph.AddEdge(Edge(firstNode, lastNode, weight));
            graph.AddEdge(Edge(lastNode, firstNode, weight));
        }

        isDrawingEdge = true;
    }
}

void MainWindow::mouseReleaseEvent(QMouseEvent* event)
{
    isDrawingNode = false;
    isDrawingEdge = false;

    if (event->button() == Qt::RightButton)
    {
        // Add node
        QPoint point = event->pos();
        addNode(point);
    }
    else
        if (event->button() == Qt::LeftButton)
        {
            QPoint point = event->pos();
            Node foundNode;

            std::vector<Node> nodes = graph.GetNodes();
            for (const Node& node : nodes)
            {
                if (abs(node.GetCoordinates().x() - point.x()) < 2 * Node::radius &&
                    abs(node.GetCoordinates().y() - point.y()) < 2 * Node::radius)
                {
                    foundNode = node;
                    break;
                }
            }

            if (foundNode.GetValue() == -1)
            {
                if (firstNode.GetValue() != -1 && graph.ExistsNodeCloseTo(Node(point)) == false)
                {
                    // Move the node to a given position
                    moveNode(point);
                }

                return;
            }

            if (firstNode.GetValue() == -1)
            {
                firstNode = foundNode;
                ui->firstNodeLabel->setText("First node: " + QString::number(firstNode.GetValue()));
            }
            else
            {
                lastNode = foundNode;
                ui->lastNodeLabel->setText("Last node: " + QString::number(lastNode.GetValue()));

                timer = new QTimer(this);
                timer->setSingleShot(true);
                timer->start(1000);

                connect(timer, &QTimer::timeout, this, [&]() {
                    if (isBfsRunning)
                    {
                        // qDebug(QString::number(firstNode.GetValue()).toStdString().c_str());
                        // qDebug(QString::number(lastNode.GetValue()).toStdString().c_str());

                        std::vector<int> path = graph.BFS(firstNode.GetValue(), lastNode.GetValue());
                        isBfsRunning = false;

                        if (path.empty())
                            QMessageBox::information(this, "BFS", "No path found!");
                    }
                    else
                    {
                        // Add edge
                        addEdge();
                    }

                    firstNode = Node();
                    lastNode = Node();

                    ui->firstNodeLabel->setText("First node: " + QString::number(firstNode.GetValue()));
                    ui->lastNodeLabel->setText("Last node: " + QString::number(lastNode.GetValue()));
                    update();
                });


            }
        }

    graph.GenerateAdjacencyMatrix();
    graph.PrintAdjacencyMatrix();

    graph.GenerateAdjacencyList();
    graph.PrintAdjacencyLists();
}


void MainWindow::drawNode(const Node &node, Qt::GlobalColor color)
{
    QPainter painter(this);

    qreal x = node.GetCoordinates().x();
    qreal y = node.GetCoordinates().y();

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
    QPen pen(color, 2);

    QPainter painter(this);
    painter.setPen(pen);

    QLineF straightLine(edge.GetFirstNode().GetCoordinates(), edge.GetLastNode().GetCoordinates());
    straightLine.setLength(straightLine.length() - Node::radius);

    qreal mX = (edge.GetFirstNode().GetCoordinates().x() + straightLine.p2().x()) / 2;
    qreal mY = (edge.GetFirstNode().GetCoordinates().y() + straightLine.p2().y()) / 2;

    qreal dX = edge.GetLastNode().GetCoordinates().x() - edge.GetFirstNode().GetCoordinates().x();
    qreal dY = edge.GetLastNode().GetCoordinates().y() - edge.GetFirstNode().GetCoordinates().y();

    qreal distance = sqrt(pow(dX, 2) + pow(dY, 2));

    qreal cX = -distance / 10 * (dY / distance) + mX;
    qreal cY = distance / 10 * (dX / distance) + mY;

    if (graph.IsDirected())
    {
        QPointF controlPoint(cX, cY);
        QLineF ghostLine(controlPoint, edge.GetLastNode().GetCoordinates());
        ghostLine.setLength(ghostLine.length() - Node::radius);

        double angle = ::acos(ghostLine.dx() / ghostLine.length());

        if (ghostLine.dy() >= 0)
            angle = (M_PI * 2) - angle;

        qreal arrowSize = 10;
        QPointF arrowP1 = ghostLine.p2() - QPointF(sin(angle + M_PI / 3) * arrowSize, cos(angle + M_PI / 3) * arrowSize);
        QPointF arrowP2 = ghostLine.p2() - QPointF(sin(angle + M_PI - M_PI / 3) * arrowSize, cos(angle + M_PI - M_PI / 3) * arrowSize);

        QPainterPath curvedLine;
        curvedLine.moveTo(edge.GetFirstNode().GetCoordinates());
        curvedLine.quadTo(controlPoint, ghostLine.p2());

        painter.strokePath(curvedLine, pen);
        painter.drawLine(ghostLine.p2(), arrowP1);
        painter.drawLine(ghostLine.p2(), arrowP2);
    }
    else
        painter.drawLine(straightLine);

    if (graph.IsWeighted())
    {
        painter.setFont(QFont("Times", 12, QFont::Bold));
        painter.drawText(QPointF((mX + cX) / 2, (mY + cY) / 2), QString::number(edge.GetWeight()));
    }
}


void MainWindow::paintEvent(QPaintEvent*)
{
    const std::vector<Edge>& edges = graph.GetUniqueEdges();

    if (edges.size())
    {
        for (const Edge& edge : edges)
        {
            drawEdge(edge, Qt::black);
        }

        if (isDrawingEdge == true)
        {
            drawEdge(edges[edges.size() - 1], Qt::red);
        }
    }

    const std::vector<Node>& nodes = graph.GetNodes();

    if (nodes.size())
    {
        for (const Node& node : nodes)
        {
            drawNode(node, Qt::white);
        }

        if (isDrawingNode == true)
        {
            drawNode(nodes[nodes.size() - 1], Qt::red);
        }
    }
}


void MainWindow::undirectedRadioButtonClicked()
{
    graph.SetDirected(false);
    graph.Clear();

    update();
}


void MainWindow::directedRadioButtonClicked()
{
    graph.SetDirected(true);
    graph.Clear();

    update();
}


void MainWindow::unweightedRadioButtonClicked()
{
    graph.SetWeighted(false);
    graph.Clear();

    ui->inputWeightLabel->setHidden(true);
    ui->inputWeightTextEdit->setHidden(true);
    update();
}


void MainWindow::weightedRadioButtonClicked()
{
    graph.SetWeighted(true);
    graph.Clear();

    ui->inputWeightLabel->setHidden(false);
    ui->inputWeightTextEdit->setHidden(false);
    update();
}


void MainWindow::on_actionBreadthFirstSearch_triggered()
{
    isBfsRunning = true;
}

